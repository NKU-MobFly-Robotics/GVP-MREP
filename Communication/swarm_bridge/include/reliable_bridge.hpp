/**
 * @file reliable_bridge.hpp
 * @author Haojia Li (wisderek@stumail.neu.edu.cn)
 * @brief Reliable bridge for ros data transfer in unstable network.
 * It will estiblish the connections as peer to peer mode.
 * It will reconnect each other autonomously.
 * It can guarantee the data transfer to the device in the strict order.
 * It has a queue for sending data asynchronously.
 *
 * Note: This program relies on ZMQ amd ZMQPP.
 * sudo apt install libzmqpp-dev
 *
 * Core Idea: It would create the sending and receving thread for each devices
 * and processing asynchronously. The index number would correspond to the
 * resource for one device.
 *
 * @version 0.1
 * @date 2021-08-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef __RELIABLE_BRIDGE__
#define __RELIABLE_BRIDGE__
#include <boost/shared_array.hpp>
#include "atomic"
#include "condition_variable"
#include "deque"
#include "map"
#include "mutex"
#include "ros/ros.h"
#include "stdio.h"
#include "stdlib.h"
#include "thread"
#include "time.h"
#include "zmq.hpp"
#include "zmqpp/zmqpp.hpp"

using namespace std;
namespace ser = ros::serialization;

/**
 * @brief overload operator << for add SerializedMessage to zmqpp::message
 */
zmqpp::message &operator<<(zmqpp::message &in,
                           ros::SerializedMessage const &msg) {
  in.add_raw(reinterpret_cast<void const *>(msg.buf.get()), msg.num_bytes);
  return in;
}

class ReliableBridge {
// Define print
#define OUTPUT_MSG 2

#if (OUTPUT_MSG == 1)
#define print_info(...) ROS_INFO(__VA_ARGS__)
#define print_warning(...)
#elif (OUTPUT_MSG == 2)
#define print_info(...) ROS_INFO(__VA_ARGS__)
#define print_warning(...) ROS_WARN(__VA_ARGS__)
#define print_error(...) ROS_ERROR(__VA_ARGS__)
#else
#define print_info(...)
#define print_warning(...)
#endif

 public:
  // -------------------- Construct and Destroy --------------------

  /**
   * @brief Construct a new Reliable Bridge object
   *
   * @param self_ID self ID (0-99)
   * @param IP_list other device IPs
   * @param ID_list other device ID numbers (0-99)
   * @param Queue_size max send message queue for one device
   */
  ReliableBridge(const int self_ID, vector<string> &IP_list,
                 vector<int> &ID_list, uint Queue_size = 10000)
      : self_id_(self_ID),
        ip_list_(IP_list),
        id_list_(ID_list),
        queue_size_(Queue_size),
        thread_flag_(true) {
    // ---------- Check IP and ID ----------
    int maxValue = *max_element(id_list_.begin(), id_list_.end());
    int minValue = *min_element(id_list_.begin(), id_list_.end());

    // 1) Equal size of IP and ID
    if (ip_list_.size() != id_list_.size()) {
      print_error("[TCP Bridge]: IP doesn't match ID.");
      assert(ip_list_.size() == id_list_.size());
      return;
    }

    // 2) Size of IPs
    if (id_list_.size() > 100) {
      print_error("[TCP Bridge]: Bridge only supports up to 100 devices.");
      assert(id_list_.size() <= 100);
      return;
    }

    // 3) Range of IDs
    if (maxValue > 100 || minValue < 0 || self_ID > 100 || self_ID < 0) {
      print_error("[TCP Bridge]: ID is invalid.");
      assert(maxValue <= 100 && minValue > 0 && self_ID <= 100 && self_ID > 0);
      return;
    }

    // 4) Same ID
    if (containsDuplicate(id_list_) == true) {
      print_error("[TCP Bridge]: ID list has duplicate!");
      assert(containsDuplicate(id_list_) == false);
      return;
    }

    // 5) Remove self ID and IP in list_
    const auto &self_pos =
        std::find(id_list_.begin(), id_list_.end(), self_id_);
    if (self_pos != id_list_.end()) {
      int ind = self_pos - id_list_.begin();
      id_list_.erase(self_pos);
      ip_list_.erase(ip_list_.begin() + ind);
    }

    // ---------- Connect Others ----------
    for (size_t i = 0; i < ip_list_.size(); i++) {
      // ***** build index map *****
      id_remap_.emplace(std::make_pair(id_list_[i], i));

      // ***** build senders to each device *****
      // The bind port number is 30000+self_id_*100+other_device_id.
      string url = "tcp://*:" + to_string(30000 + self_id_ * 100 + id_list_[i]);
      print_info("[TCP Bridge]: Bind: %s", url.c_str());  // bind is server
      unique_ptr<zmqpp::socket> sender(
          new zmqpp::socket(context_, zmqpp::socket_type::push));
      sender->bind(url);  // for others connection
      senders_.emplace_back(std::move(sender));

      // ***** build receivers of each device *****
      // The receive port number is 30000+other_device_id*100+self_ID.
      url = "tcp://" + ip_list_[i] + ":" +
            to_string(30000 + id_list_[i] * 100 + self_id_);
      print_info("[TCP Bridge]: Connect: %s",
                 url.c_str());  // connect is client
      unique_ptr<zmqpp::socket> receiver(
          new zmqpp::socket(context_, zmqpp::socket_type::pull));
      receiver->connect(url);  // connect to others
      receivers_.emplace_back(std::move(receiver));

      // ***** initialize callbacks and send_datas of each device *****
      callback_list_.emplace_back(
          new map<string, function<void(int, ros::SerializedMessage &)>>());
      send_data_.emplace_back(
          new std::deque<pair<string, ros::SerializedMessage>>());
      sender_mutex_.emplace_back(new mutex());       // locker
      cond_.emplace_back(new condition_variable());  // notice sender to send.

      // ***** create send and receive threads of each device *****
      send_threads_.emplace_back(
          std::thread(&ReliableBridge::transportMsgThread, this, i));
      recv_threads_.emplace_back(
          std::thread(&ReliableBridge::recvThread, this, i));
    }
  }

  /**
   * @brief Destroy the Reliable Bridge object
   *
   */
  // ~ReliableBridge();

  //
  // -------------------- Receive Functions --------------------

  /**
   * @brief register a callback for receiving data from a specific device.
   * For processing data, you should register a callback at the beginning.
   * Cation: The callback will be used in other threads, so ensure the callback
   * can be reentrant!! Please use local variables. When you want to write into
   * global variables, it is better to use locker to protect them.\
   *
   * @param ID :receive data from specific device (MUST IN ID LIST)
   * @param topic_name :specific topic
   * @param callback :callback function, like `void callbackSample(int ID,
   * ros::SerializedMessage& m)`
   * @example
   * void callbackSample(int ID, ros::SerializedMessage& m)
   *      {
   *          geometry_msgs::Point msg;
   *          ros::serialization::deserializeMessage(m,msg);
   *          //ID is which device send this message.
   *          //msg is the ros message.
   *      }
   */
  void registerCallback(
      int ID, string topic_name,
      function<void(int, ros::SerializedMessage &)> callback) {
    int ind;
    try {
      ind = id_remap_.at(ID);
      callback_list_[ind]->emplace(topic_name, callback);
    } catch (const std::exception &e) {
      print_error("[TCP Bridge]: ID %d is not in the list!", ID);
      return;
    }
  }

  /**
   * @brief register a callback for receiving data from all other devices.
   * For processing data, you should register a callback at the beginning.
   * Cation: The callback will be used in other threads, so ensure the callback
   * can be reentrant!! Please use local variables. When you want to write into
   * global variables, it is better to use locker to protect them.
   *
   * @param topic_name :specific topic
   * @param callback :callback function, like `void callbackSample(int ID,
   * ros::SerializedMessage& m)`
   * @example
   * void callbackSample(int ID, ros::SerializedMessage& m)
   *      {
   *          geometry_msgs::Point msg;
   *          ros::serialization::deserializeMessage(m,msg);
   *          //ID is which device send this message.
   *          //msg is the ros message.
   *      }
   */
  void registerCallbackForAll(
      string topic_name,
      function<void(int, ros::SerializedMessage &)> callback) {
    for (size_t i = 0; i < id_list_.size(); i++)
      registerCallback(id_list_[i], topic_name, callback);
  }

  //
  // -------------------- Send Functions --------------------

  /**
   * @brief send message to a specific device asynchronously.
   * This function will push the data in queue (send_data_) and the send thread
   * will be notified to send the data immediately.
   *
   * @tparam T <message type>
   * @param ID :Which device do you want to send (MUST IN ID LIST).
   * @param topic_name :topic name
   * @param msg :ros message you want to send
   * @return int :0 for no error, -1 for queue is full
   * @example
   * std_msgs::String text;
   * sendMsgToOne(1,"/topic/str",text);
   */
  template <typename T>
  int sendMsgToOne(int ID, string topic_name, T &msg) {
    int ind;
    try {
      ind = id_remap_.at(ID);
    } catch (const std::exception &e) {
      print_warning("[TCP Bridge]: ID %d is not in the list!", ID);
      return -2;
    }
    auto &buffer = send_data_[ind];
    if (buffer->size() > queue_size_) {  // buffer is full
      print_warning("[TCP Bridge]: ID:%d Send buffer is full", ID);
      return -1;
    }

    // update send_data
    {
      unique_lock<mutex> locker(*sender_mutex_.at(ind));
      buffer->emplace_back(
          make_pair(topic_name, ser::serializeMessage<T>(msg)));
      locker.unlock();
      cond_[ind]->notify_all();
    }
    return 0;
  }

  /**
   * @brief send message to a all device asynchronously (not include self)
   * This function will push the data in queue and the send thread will send the
   * data immediately.
   *
   * @tparam T <message type>
   * @param topic_name :topic name
   * @param msg :ros message you want to send
   * @return int :0 for no error, <0 for queue is full
   * @example
   * std_msgs::String text;
   * sendMsgToAll("/topic/str",text);
   */
  template <typename T>
  int sendMsgToAll(string topic_name, T &msg) {
    int err_code = 0;
    for (size_t i = 0; i < id_list_.size(); i++) {
      err_code += sendMsgToOne<T>(id_list_[i], topic_name, msg);
    }
    return err_code;
  }

  //
  // -------------------- Stop --------------------

  /**
   * @brief Stop the TCP threads (send and recv).
   */
  void StopThread() {
    thread_flag_ = false;
    for (size_t i = 0; i < ip_list_.size(); i++)  // close all senders
      senders_[i]->close();

    for (std::thread &th : recv_threads_) {  // close recv threads
      if (th.joinable()) {
        pthread_cancel(th.native_handle());
        std::cout << "[TCP Bridge]: rec thread stopped" << std::endl;
      } else
        std::cout << "[TCP Bridge]: cannot join rec thread: " << th.get_id()
                  << std::endl;
    }

    for (std::thread &th : send_threads_) {  // close send threads
      if (th.joinable()) {
        pthread_cancel(th.native_handle());
        std::cout << "[TCP Bridge]: send thread stopped" << std::endl;
      } else
        std::cout << "[TCP Bridge]: cannot join send thread: " << th.get_id()
                  << std::endl;
    }
  }

 private:
  // -------------------- Threads --------------------
  /**
   * @brief sender thread.
   * Send data in queue asynchronously.
   *
   * @param index ：thread index number
   */
  void transportMsgThread(int index) {
    int ind = index;
    while (thread_flag_) {
      // wait for update of send_data
      unique_lock<mutex> locker(*sender_mutex_.at(ind));
      cond_[ind]->wait(locker);  // active when notified
      auto &buffer = send_data_[ind];
      locker.unlock();  // release in time

      // broadcast all data (one by one), include not only one msg
      while (!buffer->empty() && thread_flag_) {
        auto &data = buffer->front();
        zmqpp::message send_array;

        // data, topic << bytes << msg
        send_array << data.first << data.second.num_bytes << data.second;
        // block here, wait for sending
        if (senders_[ind]->send(send_array, false)) {
          unique_lock<mutex> locker2(*sender_mutex_.at(ind));
          buffer->pop_front();  // delete the first data
          locker2.unlock();
        }
      }
    }
  }

  /**
   * @brief receiver thread.
   * Receive data and call the callback function asynchronously.
   *
   * @param index ：thread index number
   */
  void recvThread(int index) {
    int ind = index;
    int ID = id_list_[ind];
    while (thread_flag_) {
      zmqpp::message recv_array;

      if (receivers_[ind]->receive(recv_array, false)) {
        string topic_name;
        size_t data_len;
        ros::SerializedMessage msg_ser;

        // unpack meta data
        recv_array >> topic_name >> data_len;
        // unpack ros messages
        msg_ser.buf.reset(new uint8_t[data_len]);
        memcpy(msg_ser.buf.get(),
               static_cast<const uint8_t *>(
                   recv_array.raw_data(recv_array.read_cursor())),
               data_len);
        recv_array.next();  // move read_cursor for next part.
        msg_ser.num_bytes = data_len;
        msg_ser.message_start = msg_ser.buf.get() + 4;

        // find the callback of this topic
        auto &topic_cb = callback_list_[ind];
        const auto &iter = topic_cb->find(topic_name);
        if (iter != topic_cb->end())
          iter->second(ID, msg_ser);  // go into callback;
      }
      usleep(1);
    }
  }

  // -------------------- Helpers --------------------
  /**
   * @brief Check whether the v contains duplicate (same element)
   *
   * @tparam T <type>
   * @param v vector
   * @return true :Contain Duplicate
   * @return false :No Duplicate
   */
  template <typename T>
  static bool containsDuplicate(const vector<T> &v) {
    unordered_set<T> s(v.size() * 2);
    for (auto x : v)
      if (!s.insert(x).second) return true;
    return false;
  }

  // -------------------- Parameters --------------------

  bool thread_flag_;  // whether running threads

  zmqpp::context_t context_;  // context

  // for communicating with each device,each device is allocated with
  // a sender, a receiver, a send_data queue for data transfer, a sender_mutex,
  // a condition_variable for notice the sender, a send_thread, a recv_thread,
  // a callback map for finding the callback function

  vector<unique_ptr<zmqpp::socket>> senders_;    // index senders_
  vector<unique_ptr<zmqpp::socket>> receivers_;  // index receivers_

  // mutiple threads
  vector<unique_ptr<mutex>> sender_mutex_;       // mutex
  vector<unique_ptr<condition_variable>> cond_;  // condition to notify mutex
  vector<std::thread> send_threads_;             // send threads
  vector<std::thread> recv_threads_;             // recv threads

  // data list
  vector<unique_ptr<std::deque<pair<string, ros::SerializedMessage>>>>
      send_data_;

  // callback list, index->topic_name->callback
  vector<unique_ptr<map<string, function<void(int, ros::SerializedMessage &)>>>>
      callback_list_;

  vector<int> id_list_;     // all ID
  vector<string> ip_list_;  // all IP
  map<int, int> id_remap_;  // map ID -> index
  const int self_id_;       // self id
  const uint queue_size_;
};

#endif
