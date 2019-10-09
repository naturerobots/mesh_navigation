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
      char *&data)
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
    if(float_channels.find("vertices") != float_channels.end())
      return float_channels["vertices"];

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
        // for (int i = 0; i < size * width; i++)
        // {
        //   std::cout << float_data[i] << "; ";
        // }
        // std::cout << std::endl;

        auto channel = lvr2::FloatChannel(size, width);
        memcpy(channel.dataPtr().get(), float_data, size*width*sizeof(float));
        std::cout << "done." <<std::endl;
        return channel;
      }
    }
    return lvr2::FloatChannelOptional();
  }

  lvr2::IndexChannelOptional MeshClient::getIndices()
  {
    if(index_channels.find("face_indices") != index_channels.end())
      return index_channels["face_indices"];


    auto res = http_client.Post(srv_endpoint.c_str(), buildJson("face_indices"), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      bool ok = parseByteDataString(res->body, type, size, width, data) && type == Type::UINT;
      std::cout << "ok: " << ok << " type: " << (((int)type) & 0xFF) << " size: " << size << " width: " << width << std::endl;

      if(ok)
      {
        unsigned int* index_data = reinterpret_cast<unsigned int*>(data);

        // for (int i = 0; i < size * width; i++)
        // {
        //   std::cout << index_data[i] << "; ";
        // }

        auto channel = lvr2::IndexChannel(size, width);
        memcpy(channel.dataPtr().get(), index_data, size*width*sizeof(lvr2::Index));
        std::cout << "done." <<std::endl;

        // for (int i = 0; i < size * width; i++)
        // {
        //   std::cout << channel.dataPtr()[i] << "; ";
        // }

        return channel;
      }
    }
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
    if(float_channels.find(name) != float_channels.end()){
      channel = float_channels[name];
      return true;
    }

    auto res = http_client.Post(srv_endpoint.c_str(), buildJson(name, group), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(parseByteDataString(res->body, type, size, width, data) && type == Type::FLOAT)
      {
        float* float_data = reinterpret_cast<float*>(data);
        channel = lvr2::FloatChannel(size, width);
        memcpy(channel.get().dataPtr().get(), float_data, size*width*sizeof(float));
        return true;
      }
    }
    return false;
  }

  bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::IndexChannelOptional& channel)
  {
    if(index_channels.find(name) != index_channels.end())
    {
      channel = index_channels[name];
      return true;
    }

    auto res = http_client.Post(srv_endpoint.c_str(), buildJson(name, group), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(!parseByteDataString(res->body, type, size, width, data) && type == Type::UINT)
      {
        unsigned int* index_data = reinterpret_cast<unsigned int*>(data);
        channel = lvr2::IndexChannel(size, width);
        memcpy(channel.get().dataPtr().get(), index_data, size*width*sizeof(lvr2::Index));
        return true;
      }
    }
    return false;
  }

  bool MeshClient::getChannel(const std::string group, const std::string name, lvr2::UCharChannelOptional& channel)
  {
    if(uchar_channels.find(name) != uchar_channels.end()){
      channel = uchar_channels[name];
      return true;
    }

    auto res = http_client.Post(srv_endpoint.c_str(), buildJson(name, group), "application/json");
    if(res && res->status == 200)
    {
      char type;
      unsigned long size, width;
      char *data;

      if(parseByteDataString(res->body, type, size, width, data) && type == Type::UCHAR)
      {
        unsigned char* uchar_data = reinterpret_cast<unsigned char*>(data);
        channel = lvr2::UCharChannel(size, width);
        memcpy(channel.get().dataPtr().get(), uchar_data, size*width*sizeof(unsigned char));
        return true;
      }
    }
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
} /* namespace mesh_client */
