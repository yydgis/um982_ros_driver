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

#ifndef UBLOX_SERIALIZATION_H
#define UBLOX_SERIALIZATION_H

#include <ros/console.h>
#include <stdint.h>
#include <boost/call_traits.hpp>
#include <boost/format.hpp>
#include <vector>
#include <algorithm>

#include "checksum.h"

///
/// This file defines the Serializer template class which encodes and decodes
/// specific message types. 
/// The Reader class decodes messages and from a buffer and the Writer class 
/// encodes messages and writes them to a buffer.
/// It also declares macros for declaring Messages. The Message class
/// maps ROS messages types to class and message ID(s).
///


/**
 * @namespace ublox
 * This namespace is for u-blox message serialization.
 */
namespace ublox {

//! u-blox message Sync A char
static const uint8_t DEFAULT_SYNC_A = 0xB5; 
//! u-blox message Sync B char
static const uint8_t DEFAULT_SYNC_B = 0x62; 
//! Maximum payload length
static const uint32_t kMaxPayloadLength = 8184;  // == (buffer size - header length - checksum length)
//! Number of bytes in a message header (Sync chars + class ID + message ID)
static const uint8_t kHeaderLength = 6; 
//! Number of checksum bytes in the u-blox message
static const uint8_t kChecksumLength = 2; 
static const uint8_t kHeaderSkipLength = 3; 
/**
 * @brief Encodes and decodes messages.
 */
template <typename T>
struct Serializer {
  /**
   * @brief Decode the message payload from the data buffer.
   * @param data a pointer to the start of the message payload
   * @param count the number of bytes in the message payload
   * @param message the output message
   */
  static void read(const uint8_t *data, uint32_t count, 
                   typename boost::call_traits<T>::reference message);
  /**
   * @brief Get the length of the message payload in bytes.
   * 
   * @details The payload does not include the header or checksum.
   * @param message the message to get the length of
   * @return the length of the message in bytes.
   */
  static uint32_t serializedLength(
      typename boost::call_traits<T>::param_type message);
  
  /**
   * @brief Encode the message payload as a byte array.
   * @param data a buffer to fill with the message payload bytes
   * @param size the length of the buffer
   * @param message the output message
   */
  static void write(uint8_t *data, uint32_t size, 
                    typename boost::call_traits<T>::param_type message);
};

/**
 * @brief Keeps track of which class and message IDs can be decoded by a given
 * message type.
 */
template <typename T>
class Message {
 public:
  /**
   * @brief Can this message type decode a u-blox message with the given ID?
   * @param class_id the class ID of the u-blox message
   * @param message_id the message ID of the u-blox message
   * @return whether or not this message type decode the u-blox message
   */
  static bool canDecode(uint8_t class_id, uint32_t message_id) {
    return std::find(keys_.begin(), keys_.end(), 
                     std::make_pair(class_id, message_id)) != keys_.end();
  }
  
  /**
   * @brief Indicate that this message type can decode u-blox messages with the 
   * given ID
   * @param class_id the class ID of the u-blox message
   * @param message_id the message ID of the u-blox message
   */
  static void addKey(uint8_t class_id, uint32_t message_id) {
    keys_.push_back(std::make_pair(class_id, message_id));
  }

  struct StaticKeyInitializer
  {
    StaticKeyInitializer(uint8_t class_id, uint32_t message_id) { 
      Message<T>::addKey(class_id, message_id); 
    }
  };

 private:
  static std::vector<std::pair<uint8_t,uint32_t> > keys_; // uint32_t for UM982 
};

/**
 * @brief Options for the Reader and Writer for encoding and decoding messages.
 */
struct Options {
  /**
   * The default options for a u-blox message.
   */
  Options() : sync_a(DEFAULT_SYNC_A), sync_b(DEFAULT_SYNC_B), 
	      max_payload_length(kMaxPayloadLength),
              header_length(kHeaderLength), checksum_length(kChecksumLength), header_skip(kHeaderSkipLength){}
  //! The sync_a byte value identifying the start of a message
  uint8_t sync_a; 
  //! The sync_b byte value identifying the start of a message
  uint8_t sync_b; 
  //! The maximum payload length.
  uint32_t max_payload_length;
  //! The length of the message header in bytes (everything before the payload)
  uint8_t header_length; 
  //! The length of the checksum in bytes
  uint8_t checksum_length; 
  
 //! for UM982  decode ,skip  bytes
 uint8_t header_skip; // fix 3 for unicore
  /**
   * @brief Get the number of bytes in the header and footer.
   * @return the number of bytes in the header and footer
   */
  int wrapper_length() {
    return header_length + checksum_length; 
  }
};

/** 
 * @brief Decodes byte messages into u-blox ROS messages.
 */
class Reader {
 public:
  /**
   * @param data a buffer containing u-blox messages
   * @param count the size of the buffer
   * @param options A struct containing the parameters sync_a and sync_b  
   * which represent the sync bytes indicating the beginning of the message
   */
  Reader(const uint8_t *data, uint32_t count,
         const Options &options = Options()) : 
      data_(data), count_(count), found_(false),ubloxDev(true),options_(options)
  {
          unused_data_.reserve(1024);
  }

  typedef const uint8_t *iterator;

  /**
   * @brief Search the buffer for the beginning of the next u-blox message
   * @return a pointer to the start of the next u-blox message
   */
  virtual iterator search()
  {
    if (found_) next();
    // Search for a message header
    for( ; count_ > 0; --count_, ++data_) {
      if (data_[0] == options_.sync_a && 
          (count_ == 1 || data_[1] == options_.sync_b)) {
        // Ignore messages which exceed the maximum payload length
        if (length() > options_.max_payload_length) {
          // Message exceeds maximum payload length
          ROS_ERROR("U-Blox message exceeds maximum payload length %u: "
	            "0x%02x / 0x%02x", options_.max_payload_length,
		    classId(), messageId());
          continue;
        }
        //ROS_DEBUG("HIT UBX HEADER");
        break;
      }
      else {
          unused_data_.push_back(data_[0]);
      }
    }

    return data_;
  }

  /**
   * @brief Has a u-blox message been found in the buffer?
   * @returns true if A message with the correct header & length has been found
   */
  virtual bool found()
  {
    if (found_) return true;
    // Verify message is long enough to have sync chars, id, length & checksum
    if (count_ < options_.wrapper_length()) return false;
    // Verify the header bits
    if (data_[0] != options_.sync_a || data_[1] != options_.sync_b) 
      return false;
    // Verify that the buffer length is long enough based on the received
    // message length
    if (count_ < length() + options_.wrapper_length()) return false;

    found_ = true;
    //ROS_DEBUG("found UBX msg");
    return true;
  }

  /**
   * @brief Go to the start of the next message based on the received message 
   * length.
   *
   * @details Warning: Does not go to the correct byte location if the received 
   * message length is incorrect. If this is the case, search must be called.
   */
  virtual iterator next() {
    if (found()) {
      uint32_t size = length() + options_.wrapper_length();
      data_ += size; count_ -= size;
    }
    found_ = false;
    return data_;
  }

  /**
   * @brief Get the current position in the read buffer.
   * @return the current position of the read buffer
   */
  virtual iterator pos() {
    return data_;
  }

  virtual iterator end() {
    return data_ + count_;
  }

  virtual uint8_t classId() { return data_[2]; }
  // for UM982 uint8_t ->uint32_t
  virtual uint32_t messageId() { return data_[3]; }

  /**
   * @brief Get the length of the u-blox message payload.
   *
   * @details Payload length does not include the header or checksum length.
   * Determines the length from the header of the u-blox message.
   * @return the length of the message payload
   */
  virtual uint32_t length() {
    if (count_ < 6) return 0u;
    return (data_[5] << 8) + data_[4];
  }
  // not caller ?
  virtual  const uint8_t *data() { return data_ + options_.header_length; }
  
  /**
   * @brief Get the checksum of the u-blox message.
   *
   * @return the checksum of the u-blox message
   */
  virtual uint32_t checksum() { 
    ROS_DEBUG("checksum for ubx");
    uint32_t ubx_crc = 0;
    ubx_crc =  *reinterpret_cast<const uint16_t *>(data_ + options_.header_length +
                                               length()); 
    return ubx_crc;
  }

  /**
   * @brief Decode the given message.
   * @param message the output message
   * @param search whether or not to skip to the next message in the buffer
   */
  template <typename T>
  bool read(typename boost::call_traits<T>::reference message, 
            bool search = false) {
    if (search) this->search();
    if (!found()) return false; 
    if (!Message<T>::canDecode(classId(), messageId())) return false;

    if (ubloxDev) {
      //for ublox
      uint16_t chk;
      if (calculateChecksum(data_ + 2, length() + 4, chk) != this->checksum()) {
        // checksum error
        ROS_DEBUG("U-Blox read checksum error: 0x%02x / 0x%02x", classId(), 
                  messageId());
        return false;
      }
      // serialize  msg  from memory
      Serializer<T>::read(data_ + options_.header_length, length(), message);
    }
    else // for UM982
    {
      uint32_t len = length()+options_.header_length;
      uint32_t crc32 = CalculateCRC32(data_,len);
      uint32_t msg_crc = this->checksum();
      //ROS_DEBUG("unicore msg_crc=0x%08x crc32=0x%08x, len=%d",msg_crc,crc32,len);
      if(crc32!= msg_crc )
      {
        // checksum error
        ROS_ERROR("unicore msg read checksum error: 0x%02x / 0x%02x msg_crc %04x calcuted crc %04x ", classId(), 
                  messageId(),msg_crc,crc32);
        return false;
      }
      
      if (0) {
        // Print the received bytes
        std::ostringstream oss;
        for (int j = 0 ;j< len;j++)
        {
          oss << boost::format("%02x") % static_cast<unsigned int>(*(data_+j)) << " ";
          if (j == 67) { // lat_std
            oss << "\n";
          }
        }
        ROS_DEBUG("unicore Serializer  \n%s", oss.str().c_str());
      }
      len-=options_.header_skip;
      // serialize  msg , skip first four byte  of header for UM982
      //ROS_DEBUG("serializer len=%d",len);
      Serializer<T>::read(data_ + options_.header_skip,len, message);
    }
    return true;
  }

  /**
   * @brief Can the given message type decode the current message in the buffer?
   * @return whether the given message type can decode the current message in 
   * the buffer
   */
  template <typename T> 
  bool hasType() {
    if (!found()) return false;
    return Message<T>::canDecode(classId(), messageId());
  }

  /**
   * @brief Does the u-blox message have the given class and message ID?
   * @return Whether or not the u-blox message has the given class and message 
   * ID
   */
  bool isMessage(uint8_t class_id, uint32_t message_id) {
    if (!found()) return false;
    return (classId() == class_id && messageId() == message_id);
  }
  
  const std::string& getUnusedData() const { return unused_data_; }

 protected:
  //! The buffer of message bytes
  const uint8_t *data_;
  //! Unused data from the read buffer, contains nmea messages.
  std::string unused_data_;
  //! the number of bytes in the buffer, //! decrement as the buffer is read
  uint32_t count_; 
  //! Whether or not a message has been found
  bool found_; 
  //! Options representing the sync char values, etc.
  Options options_; 

  //! set true if not ublx devices, map oem msg with ubx
  bool ubloxDev;
};

/** 
 * @brief Encodes a u-blox ROS message as a byte array.
 */
class Writer {
 public:
  typedef uint8_t *iterator;

  /**
   * @brief Construct a Writer with the given buffer.
   * @param data a buffer for messages
   * @param size the size of the buffer
   * @param options options representing the message sync chars, etc.
   */
  Writer(uint8_t *data, uint32_t size, const Options &options = Options()) : 
      data_(data), size_(size), options_(options) {}

  /**
   * @brief Encode the u-blox message.
   * @param message the message to encode
   * @param class_id the u-blox class ID, defaults to the message CLASS_ID
   * @param message_id the u-blox message ID, defaults to the message MESSAGE_ID
   * @return true if the message was encoded correctly, false otherwise
   */
  template <typename T> bool write(const T& message, 
                                   uint8_t class_id = T::CLASS_ID, 
                                   uint8_t message_id = T::MESSAGE_ID) {
    // Check for buffer overflow
    uint32_t length = Serializer<T>::serializedLength(message);
    if (size_ < length + options_.wrapper_length()) {
      ROS_ERROR("u-blox write buffer overflow. Message %u / %u not written", 
                class_id, message_id);
      return false;
    }
    // Encode the message and add it to the buffer
    Serializer<T>::write(data_ + options_.header_length, 
                         size_ - options_.header_length, message);
    return write(0, length, class_id, message_id);
  }

  /**
   * @brief Wrap the encoded message payload with a header and checksum and
   * add it to the buffer.
   * @param message the encoded message payload (no header or checksum)
   * @param length the length of the message payload
   * @param class_id the u-blox class ID
   * @param message_id the u-blox message ID
   * @return true if the message was encoded correctly, false otherwise
   */
  bool write(const uint8_t* message, uint32_t length, uint8_t class_id, 
             uint8_t message_id) {
    if (size_ < length + options_.wrapper_length()) {
      ROS_ERROR("u-blox write buffer overflow. Message %u / %u not written", 
                class_id, message_id);
      return false;
    }
    iterator start = data_;

    // write header
    *data_++ = options_.sync_a;
    *data_++ = options_.sync_b;
    *data_++ = class_id;
    *data_++ = message_id;
    *data_++ = length & 0xFF;
    *data_++ = (length >> 8) & 0xFF;
    size_ -= options_.header_length;

    // write message
    if (message) std::copy(message, message + length, data_);
    data_ += length;
    size_ -= length;

    // write checksum
    uint8_t ck_a, ck_b;
    calculateChecksum(start + 2, length + 4, ck_a, ck_b);
    *data_++ = ck_a;
    *data_++ = ck_b;
    size_ -= options_.checksum_length;

    return true;
  }

  iterator end() {
    return data_;
  }

 private:
  //! The buffer of message bytes
  iterator data_; 
  //! The number of remaining bytes in the buffer
  /*! Decrements as messages are written to the buffer */
  uint32_t size_; 
  //! Options representing the sync char values, etc.
  Options options_; 
};

/** 
 * @brief Decodes byte messages into unicore ROS messages.
 */
class ReaderUnicore : public Reader{
 public:
  /**
   * @param data a buffer containing Unicore messages
   * @param count the size of the buffer
   * @param options A struct containing the parameters sync_a and sync_b  
   * which represent the sync bytes indicating the beginning of the message
   */
  ReaderUnicore(const uint8_t *data, uint32_t count, 
         const Options &options = Options()) : Reader(data,count,options)
  {
          unused_data_.reserve(1024);
          //update opions base one unicore
          options_.sync_a = 0xAA;
          options_.sync_b = 0x44;
          options_.header_skip = 3;
          // options.header_length is  UMOEN headlen=24  UMBIN headlen=28
          options_.header_length =  28;// default
          options_.checksum_length = 4;
          ubloxDev = false;
          //ROS_INFO(" create readerunicore for UM982");
  }

  //typedef const uint8_t *iterator;

  /**
   * @brief Search the buffer for the beginning of the next 
   *        unicore message
   * @return a pointer to the start of the next unicore message
   */
  iterator search()
  {
    if (found_) next();
    // Search for a message header
    for( ; count_ > 0; --count_, ++data_) {
      if (data_[0] == options_.sync_a && 
          (count_ == 1 || data_[1] == options_.sync_b)) {
        // Ignore messages which exceed the maximum payload length
        if (length() > options_.max_payload_length) {
          // Message exceeds maximum payload length
          ROS_ERROR("unicore message exceeds maximum payload length %u: "
	            "0x%02x / 0x%02x", options_.max_payload_length,
                    classId(), messageId());
          continue;
        }
        //ROS_DEBUG("HIT UNICORE HEADER");
        break;
      }
      else {
          unused_data_.push_back(data_[0]);
      }
    }

    return data_;
  }

  /**
   * @brief Has a u-blox message been found in the buffer?
   * @returns true if A message with the correct header & length has been found
   */
  bool found()
  {
    if (found_) return true;
    // Verify message is long enough to have sync chars, id, length & checksum
    if (count_ < options_.wrapper_length()) return false;
    // Verify the header bits
    if (data_[0] != options_.sync_a || data_[1] != options_.sync_b) 
      return false;
    // Verify that the buffer length is long enough based on the received
    // message length
    // for UM980
    
    if (count_ < length() + options_.wrapper_length()) return false;

    found_ = true;
    return true;
  }

  /**
   * @brief Go to the start of the next message based on the received message 
   * length.
   *
   * @details Warning: Does not go to the correct byte location if the received 
   * message length is incorrect. If this is the case, search must be called.
   */
  iterator next() {
    if (found()) {
      uint32_t size = length() + options_.wrapper_length();
      ROS_DEBUG("ReaderUnicore NEXT size=%d",size);
      data_ += size; count_ -= size;
    }
    found_ = false;
    return data_;
  }

  /**
   * @brief Get the current position in the read buffer.
   * @return the current position of the read buffer
   */
  iterator pos() {
    return data_;
  }

  iterator end() {
    return data_ + count_;
  }
  // for unicore
  uint8_t classId() { return data_[2]; } // 0x12 or 0xb5
  // for unicore
  uint32_t messageId() {
    if (data_[2] == 0xb5) {
      return (data_[5] << 8) + data_[4];
    }
    else if (data_[2] == 0x12)
    {
      return (data_[5] << 8) + data_[4];
    }
    else
    {
      ROS_INFO("worng sync3 for unicore:0x%02x",data_[2] );
      return 0;
    }
  }
  uint8_t const *data() { return data_ + options_.header_skip; }
  /**
   * @brief Get the length of the u-blox message payload.
   *
   * @details Payload length does not include the header or checksum length.
   * Determines the length from the header of the u-blox message.
   * @return the length of the message payload
   */
  uint32_t length() {
    if (count_ < 10) return 0u;
    if (data_[2] == 0xb5) {
      //ROS_INFO("Header is UINOCRE OEM");
      // for UMOEM
      options_.header_length = 24;
      return (data_[7] << 8) + data_[6];
    }
    else if (data_[2] == 0x12) {
      //ROS_INFO("Header is UINOCRE BIN");
      // for UMBIN
      options_.header_length = 28;
      return (data_[9] << 8) + data_[8];
    }
    else
    {
      ROS_ERROR("unicore message wrong sync3 = 0x%02x ",data_[2]);
      return 0u;
    }
  }
  
  /**
   * @brief Get the checksum of the unicore message.
   *
   * @return the checksum of the unicore message
   */
  uint32_t checksum() { 
    const uint8_t *pcrc =  data_ + options_.header_length + length();
                                              
    uint32_t crc =  *reinterpret_cast<const uint32_t *>(pcrc); 
    return crc;
  }

  uint32_t headLen()
  {
    return options_.header_length;
  }
};

} // namespace ublox

// Use to declare u-blox messages and message serializers
#define DECLARE_UBLOX_MESSAGE(class_id, message_id, package, message) \
  template class ublox::Serializer<package::message>; \
  template class ublox::Message<package::message>; \
  namespace package { namespace { \
    static const ublox::Message<message>::StaticKeyInitializer static_key_initializer_##message(class_id, message_id); \
  } } \

// Use for messages which have the same structure but different IDs, e.g. INF
// Call DECLARE_UBLOX_MESSAGE for the first message and DECLARE_UBLOX_MESSAGE_ID
// for following declarations
#define DECLARE_UBLOX_MESSAGE_ID(class_id, message_id, package, message, name) \
  namespace package { namespace { \
    static const ublox::Message<message>::StaticKeyInitializer static_key_initializer_##name(class_id, message_id); \
  } } \


// use implementation of class Serializer in "serialization_ros.h"
#include "serialization_ros.h"

#endif // UBLOX_SERIALIZATION_H
