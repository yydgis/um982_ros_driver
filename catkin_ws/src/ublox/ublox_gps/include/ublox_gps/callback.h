//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_CALLBACK_H
#define UBLOX_GPS_CALLBACK_H

#include <ros/console.h>
#include <ublox/serialization/ublox_msgs.h>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

#include <fstream>

namespace ublox_gps {

/**
 * @brief A callback handler for a u-blox message.
 */
class CallbackHandler {
 public:
  /**
   * @brief Decode the u-blox message.
   */
  virtual void handle(ublox::Reader& reader) = 0;

  /**
   * @brief Wait for on the condition.
   */
  bool wait(const boost::posix_time::time_duration& timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    return condition_.timed_wait(lock, timeout);
  }

 protected:
  boost::mutex mutex_; //!< Lock for the handler
  boost::condition_variable condition_; //!< Condition for the handler lock
};

/**
 * @brief A callback handler for a u-blox message.
 * @typedef T the message type
 */
template <typename T>
class CallbackHandler_ : public CallbackHandler {
 public:
  typedef boost::function<void(const T&)> Callback; //!< A callback function

  /** 
   * @brief Initialize the Callback Handler with a callback function
   * @param func a callback function for the message, defaults to none
   */
  CallbackHandler_(const Callback& func = Callback()) : func_(func) {}
  
  /**
   * @brief Get the last received message.
   */
  virtual const T& get() { return message_; }

  /**
   * @brief Decode the U-Blox message & call the callback function if it exists.
   * @param reader a reader to decode the message buffer
   */
  void handle(ublox::Reader& reader) {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_DEBUG("handle read for class_id[%02x] msg_id[%04x]",reader.classId(),reader.messageId());
    try {
      if (!reader.read<T>(message_)) {
        ROS_DEBUG_COND(debug >= 2, 
                       "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)", 
                       static_cast<unsigned int>(reader.classId()),
                       static_cast<unsigned int>(reader.messageId()),
                       reader.length());
        condition_.notify_all();
        return;
      }
    } catch (std::runtime_error& e) {
      ROS_DEBUG_COND(debug >= 2, 
                     "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)", 
                     static_cast<unsigned int>(reader.classId()),
                     static_cast<unsigned int>(reader.messageId()),
                     reader.length());
      condition_.notify_all();
      return;
    }
    //do ros publish callback
    if (func_) func_(message_);
    condition_.notify_all();
  }
  
 private:
  Callback func_; //!< the callback function to handle the message
  T message_; //!< The last received message
};

/**
 * @brief Callback handlers for incoming u-blox messages.
 */
class CallbackHandlers {
 public:
  /**
   * @brief Add a callback handler for the given message type.
   * @param callback the callback handler for the message
   * @typedef.a ublox_msgs message with CLASS_ID and MESSAGE_ID constants
   */
  template <typename T>
  void insert(typename CallbackHandler_<T>::Callback callback) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                     boost::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Add a callback handler for the given message type and ID. This is 
   * used for messages in which have the same structure (and therefore msg file)
   * and same class ID but different message IDs. (e.g. INF, ACK)
   * @param callback the callback handler for the message
   * @param message_id the ID of the message
   * @typedef.a ublox_msgs message with a CLASS_ID constant
   */
  template <typename T>
  void insert(
      typename CallbackHandler_<T>::Callback callback, 
      unsigned int message_id) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    callbacks_.insert(
      std::make_pair(std::make_pair(T::CLASS_ID, message_id),
                     boost::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Add a callback handler for nmea messages
   * @param callback the callback handler for the message
   */
  void set_nmea_callback(boost::function<void(const std::string&)> callback) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    callback_nmea_ = callback;
  }

  /**
   * @brief Calls the callback handler for the message in the reader.
   * @param reader a reader containing a u-blox message
   */
  void handle(ublox::Reader& reader) {
    // Find the callback handlers for the message & decode it
    boost::mutex::scoped_lock lock(callback_mutex_);
    // key is current asio got msg 
    Callbacks::key_type key =
        std::make_pair(reader.classId(), reader.messageId());
    //ROS_DEBUG("classId[0x%02x] messageId[0x%04x]",reader.classId(),reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key);
         callback != callbacks_.upper_bound(key); ++callback)
    {
      //ROS_DEBUG("fond serialization for classId[0x%02x] messageId[0x%04x]",reader.classId(),reader.messageId());
      // the read func maybe callback, if subscrible the msg, call
      callback->second->handle(reader);
    }
  }

  /**
   * @brief Calls the callback handler for the nmea messages in the reader.
   * @param reader a reader containing an nmea message
   */
  void handle_nmea(ublox::Reader& reader) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    if(callback_nmea_.empty())
        return;

    const std::string& buffer = reader.getUnusedData();
    size_t nmea_start = buffer.find('$', 0);
    size_t nmea_end = buffer.find('\n', nmea_start);
    while(nmea_start != std::string::npos && nmea_end != std::string::npos) {
        std::string sentence = buffer.substr(nmea_start, nmea_end - nmea_start + 1);
        callback_nmea_(sentence);

        nmea_start = buffer.find('$', nmea_end+1);
        nmea_end = buffer.find('\n', nmea_start);
    }
  }

  /**
   * @brief Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename T>
  bool read(T& message, const boost::posix_time::time_duration& timeout) {
    bool result = false;
    // Create a callback handler for this message
    callback_mutex_.lock();
    CallbackHandler_<T>* handler = new CallbackHandler_<T>();
    Callbacks::iterator callback = callbacks_.insert(
      (std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID),
                      boost::shared_ptr<CallbackHandler>(handler))));
    callback_mutex_.unlock();

    // Wait for the message
    if (handler->wait(timeout)) {
      message = handler->get();
      result = true;
    }
    
    // Remove the callback handler
    callback_mutex_.lock();
    callbacks_.erase(callback);
    callback_mutex_.unlock();
    return result;
  }

  void open_log_file(std::string dir)
  {
        time_t t = time(NULL);
        struct tm time_struct = *localtime(&t);

        std::stringstream filename;
        filename.width(4); filename.fill('0');
          filename << time_struct.tm_year + 1900;
          filename.width(0); filename << '_';
        filename.width(2); filename.fill('0');
          filename << time_struct.tm_mon  + 1;
          filename.width(0); filename << '_';
        filename.width(2); filename.fill('0');
          filename << time_struct.tm_mday;
          filename.width(0); filename << '_';
        filename.width(2); filename.fill('0');
          filename << time_struct.tm_hour;
        filename.width(2); filename.fill('0');
          filename << time_struct.tm_min ;
        filename.width(0); filename << ".log";
        file_name_ = dir + filename.str();

        try {
            file_handle_.open(file_name_);
            ROS_INFO("Logging raw data to file \"%s\"",
              file_name_.c_str());
        } catch(const std::exception& e) {
            ROS_ERROR("Can't log raw data to file. "
              "Can't create file \"%s\".", file_name_.c_str());
        }
  }
  /**
   * @brief Processes u-blox messages in the given buffer & clears the read
   * messages from the buffer.
   * @param data the buffer of u-blox messages to process
   * @param size the size of the buffer
   */
  // asio worker will call readCallback after device update data
  void readCallback(unsigned char* data, std::size_t& size) {

#if 0
    ublox::Reader reader(data, size);
    // Read all U-Blox messages in buffer
    while (reader.search() != reader.end() && reader.found()) {
      if (debug >= 3) {
        // Print the received bytes
        std::ostringstream oss;
        for (ublox::Reader::iterator it = reader.pos();
             it != reader.pos() + reader.length() + 8; ++it)
          oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
        ROS_DEBUG("U-blox: reading %d bytes\n%s", reader.length() + 8, 
                 oss.str().c_str());
      }

      handle(reader);
    }
    handle_nmea(reader);
    // delete read bytes from ASIO input buffer
    std::copy(reader.pos(), reader.end(), data); // move the remain buffer to header
    size -= reader.pos() - data;
#endif
    //for UM982
#if 1
      //ROS_DEBUG("asio read:%ld from %p",size,(void*)data);

      static int cnt  = 0;
      if (debug >= 4) {
          if (!file_handle_.is_open())  {
              // create file 
              open_log_file("/data/ublox/");
          }
          if (cnt++ > 3) {
              if (!file_handle_.is_open())  {
                  // create file 
                  open_log_file("/data/ublox/");
              }
              else
              {
                try {
                    file_handle_.write((const char*)data,size);
                    // file_handle_.flush();
                } catch(const std::exception& e) {
                    ROS_WARN("Error writing to file \"%s\"", file_name_.c_str());
                }
              }
          }
      }
      ublox::ReaderUnicore readerUnicore(data, size);
      bool unicore_msg = false;
    // Read all U-Blox messages in buffer
    while (readerUnicore.search() != readerUnicore.end() && readerUnicore.found()) {
      if (debug >= 4) {
        // Print the received bytes
        std::ostringstream oss;
        for (ublox::Reader::iterator it = readerUnicore.pos();
             it != readerUnicore.pos() + readerUnicore.length()+readerUnicore.headLen()+4; ++it)
          oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
        ROS_DEBUG("unicore: reading %d bytes\n%s", readerUnicore.length()+readerUnicore.headLen()+4, 
                 oss.str().c_str());
      }
      //ROS_DEBUG("pos1=%p",readerUnicore.pos());
      handle(readerUnicore);
      unicore_msg = true;
    }
    handle_nmea(readerUnicore);
    // delete read bytes from ASIO input buffer
    std::copy(readerUnicore.pos(), readerUnicore.end(), data); // move the remain buffer to header
    size -= readerUnicore.pos() - data;
    //ROS_DEBUG("remove size = %ld",readerUnicore.pos() - data);
    //size = 0;// no data remain 
#endif
    
  }

 private:
  //kime: second type uint8_t is ok for ubx ,need  uint32_t for UM982
  typedef std::multimap<std::pair<uint8_t, uint32_t>,
                        boost::shared_ptr<CallbackHandler> > Callbacks;

  // Call back handlers for u-blox messages
  Callbacks callbacks_;
  boost::mutex callback_mutex_;
  
  //! Callback handler for nmea messages
  boost::function<void(const std::string&)> callback_nmea_;

  //
    //! Filename for storing raw data
    std::string file_name_;
    //! Handle for file access
    std::ofstream file_handle_;
};

}  // namespace ublox_gps

#endif  // UBLOX_GPS_CALLBACK_H
