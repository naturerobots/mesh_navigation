/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */
#include "mesh_client/mesh_client.h"

#include <curl/curl.h>
#include <rclcpp/rclcpp.hpp>

namespace mesh_client
{
MeshClient::MeshClient(const std::string& server_url, const std::string& server_username, const std::string& server_password, const std::string& mesh_layer, rclcpp::Logger logger)
  : server_url_(server_url)
  , server_username_(server_username)
  , server_password_(server_password)
  , mesh_layer_(mesh_layer)
  , logger_(logger)
{
}

void MeshClient::setBoundingBox(float min_x, float min_y, float min_z, const float max_x, const float max_y,
                                const float max_z)
{
  bounding_box_[0] = min_x;
  bounding_box_[1] = min_y;
  bounding_box_[2] = min_z;
  bounding_box_[3] = max_x;
  bounding_box_[4] = max_y;
  bounding_box_[5] = max_z;
}

void MeshClient::addFilter(std::string channel, float min_value, float max_value)
{
  mesh_filters_[channel] = std::make_pair(min_value, max_value);
}

std::string MeshClient::buildJson(const std::string& attribute_name)
{
  Json::Value attr;
  attr["name"] = attribute_name;

  if (mesh_filters_.size() > 0)
  {
    Json::Value filters;
    for (auto& mesh_filter : mesh_filters_)
    {
      Json::Value filter;

      filter["attribute_name"] = mesh_filter.first;
      filter["min_val"] = mesh_filter.second.first;
      filter["max_val"] = mesh_filter.second.second;

      filters.append(filter);
    }
    attr["filters"] = filters;
  }

  Json::Value json_bb;
  json_bb["x_min"] = bounding_box_[0];
  json_bb["y_min"] = bounding_box_[1];
  json_bb["z_min"] = bounding_box_[2];
  json_bb["x_max"] = bounding_box_[3];
  json_bb["y_max"] = bounding_box_[4];
  json_bb["z_max"] = bounding_box_[5];

  Json::Value request;
  request["boundingbox"] = json_bb;
  request["attribute"] = attr;
  request["layer"] = mesh_layer_;
  Json::FastWriter fast_writer;
  return fast_writer.write(request);
}

bool parseByteDataString(std::string string, char& type, unsigned long& size, unsigned long& width, char*& data)
{
  if (string.length() < 10)
  {
    return false;
  }

  // parse body
  const char* body = string.c_str();
  type = body[0];                                                // one byte for type of message
  size = *reinterpret_cast<const unsigned long*>(body + 1);      // eight bytes for length of message
  width = *reinterpret_cast<const unsigned long*>(body + 9);     // eight bytes for amount of values per element
  data = reinterpret_cast<char*>(const_cast<char*>(body + 17));  // raw data

  return true;
}

lvr2::FloatChannelOptional MeshClient::getVertices()
{
  if (float_channels.find("vertices") != float_channels.end())
  {
    return float_channels["vertices"];
  }

  unique_ptr<std::string> str = requestChannel("vertices");

  if (str && str->size() > 17)
  {
    char type;
    unsigned long size, width;
    char* data;

    RCLCPP_DEBUG_STREAM(logger_, "Received vertices channel");

    if (parseByteDataString(*str, type, size, width, data) && type == Type::FLOAT)
    {
      float* float_data = reinterpret_cast<float*>(data);
      auto channel = lvr2::FloatChannel(size, width);
      memcpy(channel.dataPtr().get(), float_data, size * width * sizeof(float));

      return channel;
    }
  }
  RCLCPP_ERROR_STREAM(logger_, "Failed to load vertices channel!");
  return lvr2::FloatChannelOptional();
}

lvr2::IndexChannelOptional MeshClient::getIndices()
{
  if (index_channels.find("face_indices") != index_channels.end())
  {
    return index_channels["face_indices"];
  }

  unique_ptr<std::string> str = requestChannel("face_indices");

  if (str && str->size() > 17)
  {
    char type;
    unsigned long size, width;
    char* data;

    RCLCPP_DEBUG_STREAM(logger_, "Received indices channel");

    if (parseByteDataString(*str, type, size, width, data) && type == Type::UINT)
    {
      unsigned int* index_data = reinterpret_cast<unsigned int*>(data);
      auto channel = lvr2::IndexChannel(size, width);
      memcpy(channel.dataPtr().get(), index_data, size * width * sizeof(lvr2::Index));

      return channel;
    }
  }
  RCLCPP_ERROR_STREAM(logger_, "Failed to load indices channel!");
  return lvr2::IndexChannelOptional();
}

bool MeshClient::addVertices(const lvr2::FloatChannel& channel)
{
  float_channels["vertices"] = channel;
  return true;
}

bool MeshClient::addIndices(const lvr2::IndexChannel& channel)
{
  index_channels["face_indices"] = channel;
  return true;
}

bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::FloatChannelOptional& channel)
{
  std::cout << "channel " << name << std::endl;
  if (float_channels.find(name) != float_channels.end())
  {
    channel = float_channels[name];
    return true;
  }

  unique_ptr<std::string> str = requestChannel(name);

  if (str && str->size() > 17)
  {
    char type;
    unsigned long size, width;
    char* data;

    RCLCPP_DEBUG_STREAM(logger_, "Received " << name << " channel");

    if (parseByteDataString(*str, type, size, width, data) && type == Type::FLOAT)
    {
      float* float_data = reinterpret_cast<float*>(data);
      channel = lvr2::FloatChannel(size, width);
      memcpy(channel.get().dataPtr().get(), float_data, size * width * sizeof(float));
      return true;
    }
  }
  RCLCPP_ERROR_STREAM(logger_, "Failed to load " << name << " channel!");
  return false;
}

bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::IndexChannelOptional& channel)
{
  if (index_channels.find(name) != index_channels.end())
  {
    channel = index_channels[name];
    return true;
  }

  unique_ptr<std::string> str = requestChannel(name);

  if (str && str->size() > 17)
  {
    char type;
    unsigned long size, width;
    char* data;

    RCLCPP_DEBUG_STREAM(logger_, "Received " << name << " channel");

    if (parseByteDataString(*str, type, size, width, data) && type == Type::UINT)
    {
      unsigned int* index_data = reinterpret_cast<unsigned int*>(data);
      channel = lvr2::IndexChannel(size, width);
      memcpy(channel.get().dataPtr().get(), index_data, size * width * sizeof(lvr2::Index));
      return true;
    }
  }
  RCLCPP_ERROR_STREAM(logger_, "Failed to load " << name << " channel!");
  return false;
}

bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::UCharChannelOptional& channel)
{
  if (uchar_channels.find(name) != uchar_channels.end())
  {
    channel = uchar_channels[name];
    return true;
  }

  unique_ptr<std::string> str = requestChannel(name);

  if (str && str->size() > 17)
  {
    char type;
    unsigned long size, width;
    char* data;

    RCLCPP_DEBUG_STREAM(logger_, "Received " << name << " channel");

    if (parseByteDataString(*str, type, size, width, data) && type == Type::UINT)
    {
      unsigned char* uchar_data = reinterpret_cast<unsigned char*>(data);
      channel = lvr2::UCharChannel(size, width);
      memcpy(channel.get().dataPtr().get(), uchar_data, size * width * sizeof(unsigned char));
      return true;
    }
  }
  RCLCPP_ERROR_STREAM(logger_, "Failed to load " << name << " channel!");
  return false;
}

bool MeshClient::addChannel(const std::string group, const std::string name, const lvr2::FloatChannel& channel)
{
  float_channels[name] = channel;
  return true;
}

bool MeshClient::addChannel(const std::string group, const std::string name, const lvr2::IndexChannel& channel)
{
  index_channels[name] = channel;
  return true;
}

bool MeshClient::addChannel(const std::string group, const std::string name, const lvr2::UCharChannel& channel)
{
  uchar_channels[name] = channel;
  return true;
}

size_t writeFunction(void* ptr, size_t size, size_t nmemb, std::string* data)
{
  data->append((char*)ptr, size * nmemb);
  return size * nmemb;
}

std::unique_ptr<std::string> MeshClient::requestChannel(std::string channel)
{
  CURL* curl;

  curl_global_init(CURL_GLOBAL_ALL);

  curl = curl_easy_init();
  if (!curl)
  {
    curl_global_cleanup();
    return nullptr;
  }

  std::string post_body = buildJson(channel);
  curl_easy_setopt(curl, CURLOPT_URL, server_url_.c_str());

  struct curl_slist* list = curl_slist_append(list, "Content-Type: application/json");

  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, list);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post_body.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_ANY);
  std::string usr_pwd = server_username_ + ":" + server_password_;
  curl_easy_setopt(curl, CURLOPT_USERPWD, usr_pwd.c_str());

  unique_ptr<std::string> result = std::make_unique<std::string>();
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeFunction);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, result.get());

  CURLcode res = curl_easy_perform(curl);
  if (res != CURLE_OK)
  {
    std::cout << "error" << std::endl;
    curl_easy_cleanup(curl);
    return nullptr;
  }

  curl_easy_cleanup(curl);
  return result;
}

} /* namespace mesh_client */
