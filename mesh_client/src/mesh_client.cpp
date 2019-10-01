#include "mesh_client/mesh_client.h"
#include "cpp-httplib/httplib.h"

namespace mesh_client{

  MeshClient::MeshClient(
      const std::string &srv_address,
      const unsigned int &srv_port,
      const std::string &srv_endpoint)
        : srv_address(srv_address),
          srv_port(srv_port),
          srv_endpoint(srv_endpoint),
          http_client(srv_address.c_str(), srv_port)
  {

  }

  void MeshClient::setBoundingBox(
    const float min_x, const float min_y, const float min_z,
    const float max_x, const float max_y, const float max_z)
  {
    bounding_box = {min_x, min_y, min_z, max_x, max_y, max_z};
    json_bb["x_min"] = min_x;
    json_bb["y_min"] = min_y;
    json_bb["z_min"] = min_z;
    json_bb["x_max"] = max_x;
    json_bb["y_max"] = max_y;
    json_bb["z_max"] = max_z;
  }

  std::string MeshClient::buildJson(const std::string& attribute_name, const std::string& attribute_group)
  {
    Json::Value attributes;
    Json::Value attr;
    attr["attribute_name"] = attribute_name;
    attr["attribute_group"] = attribute_group;
    attr["type"] = false;
    attributes.append(attr);
    Json::Value request;
    request["compression"] = false;
    request["boundingbox"] = json_bb;
    request["attributes"] = attributes;
    Json::FastWriter fast_writer;
    return fast_writer.write(request);
  }

  bool parseByteDataString(
      const std::string& string,
      char &type,
      unsigned long &size,
      unsigned long &width,
      char *data)
  {
    if(string.length() < 10) return false;
    // parse body
    const char *body = string.c_str();
    type = body[0]; // one byte
    size = *reinterpret_cast<const unsigned long*>(body + 1); // eight bytes
    width = *reinterpret_cast<const unsigned long*>(body + 9); // eight bytes
    data = reinterpret_cast<char*>(const_cast<char*>(body+ 17));
    return true;
  }


  lvr2::FloatChannelOptional MeshClient::getVertices()
  {
    auto res = http_client.Post(srv_endpoint.c_str(), buildJson("vertices"), "application/json");
    if(res) 
      std::cout << "status: " << res->status << std::endl; 
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      bool ok = parseByteDataString(res->body, type, size, width, data) && type == Type::FLOAT;
      
      std::cout << "ok: " << ok << " type: " << (((int)type) & 0xFF) << " size: " << size << " width: " << width << std::endl;
      
      if(ok)
      {
        float* float_data = reinterpret_cast<float*>(data);
        return lvr2::FloatChannel(size, width, boost::shared_array<float>(float_data));
      }
    }
    return lvr2::FloatChannelOptional();
  }

  lvr2::IndexChannelOptional MeshClient::getIndices()
  {
    auto res = http_client.Post(srv_endpoint.c_str(), buildJson("face_indices"), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(!parseByteDataString(res->body, type, size, width, data) && type == Type::UINT)
      {
        unsigned int* index_data = reinterpret_cast<unsigned int*>(data);
        return lvr2::IndexChannel(size, width, boost::shared_array<unsigned int>(index_data));
      }
    }
    return lvr2::IndexChannelOptional();

  }

  bool MeshClient::addVertices(const lvr2::FloatChannel& channel_ptr)
  {
    return false;
  }

  bool MeshClient::addIndices(const lvr2::IndexChannel& channel_ptr)
  {
    return false;
  }

  bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::FloatChannelOptional& channel)
  {
    auto res = http_client.Post(srv_endpoint.c_str(), buildJson(name, group), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(parseByteDataString(res->body, type, size, width, data) && type == Type::FLOAT)
      {
        float* float_data = reinterpret_cast<float*>(data);
        channel = lvr2::FloatChannel(size, width, boost::shared_array<float>(float_data));
        return true;
      }
    }
    return false;
  }

  bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::IndexChannelOptional& channel)
  {
    auto res = http_client.Post(srv_endpoint.c_str(), buildJson(name, group), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(!parseByteDataString(res->body, type, size, width, data) && type == Type::UINT)
      {
        unsigned int* index_data = reinterpret_cast<unsigned int*>(data);
        channel = lvr2::IndexChannel(size, width, boost::shared_array<unsigned int>(index_data));
        return true;
      }
    }
    return false;
  }

  bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::UCharChannelOptional& channel)
  {
    auto res = http_client.Post(srv_endpoint.c_str(), buildJson(name, group), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(parseByteDataString(res->body, type, size, width, data) && type == Type::UCHAR)
      {
        unsigned char* uchar_data = reinterpret_cast<unsigned char*>(data);
        channel = lvr2::UCharChannel(size, width, boost::shared_array<unsigned char>(uchar_data));
      }
    }
  }

  bool MeshClient::addChannel(const std::string group, const std::string name, const lvr2::FloatChannel& channel)
  {
    return false;
  }

  bool MeshClient::addChannel(const std::string group, const std::string name, const lvr2::IndexChannel& channel)
  {
    return false;
  }

  bool MeshClient::addChannel(const std::string group, const std::string name, const lvr2::UCharChannel& channel)
  {
    return false;
  }
} /* namespace mesh_client */
