
#ifndef MESH_CLIENT_H_
#define MESH_CLIENT_H_

#include <string>
#include <array>
#include <lvr2/io/AttributeMeshIOBase.hpp>
#include <jsoncpp/json/json.h>
#include "cpp-httplib/httplib.h"

namespace mesh_client{
class MeshClient : public lvr2::AttributeMeshIOBase {

public:
  MeshClient(
      const std::string &srv_address,
      const unsigned int &srv_port,
      const std::string &srv_endpoint);

  void setBoundingBox(
      const float min_x, const float min_y, const float min_z,
      const float max_x, const float max_y, const float max_z
      );

  std::string buildJson(const std::string& attribute_name, const std::string& attribute_group = "");

  enum Type : char{
    UINT = 0,
    FLOAT = 1,
    UCHAR = 2
  };

  /**
   * @brief Persistence layer interface, Accesses the vertices of the mesh in the persistence layer.
   * @return An optional float channel, the channel is valid if the mesh vertices have been read successfully
   */
  lvr2::FloatChannelOptional getVertices();

  /**
   * @brief Persistence layer interface, Accesses the face indices of the mesh in the persistence layer.
   * @return An optional index channel, the channel is valid if the mesh indices have been read successfully
   */
  lvr2::IndexChannelOptional getIndices();

  /**
   * @brief Persistence layer interface, Writes the vertices of the mesh to the persistence layer.
   * @return true if the channel has been written successfully
   */
  bool addVertices(const lvr2::FloatChannel& channel_ptr);

  /**
   * @brief Persistence layer interface, Writes the face indices of the mesh to the persistence layer.
   * @return true if the channel has been written successfully
   */
  bool addIndices(const lvr2::IndexChannel& channel_ptr);

  /**
   * @brief getChannel  Reads a float attribute channel in the given group with the given name
   * @param group       The associated attribute group
   * @param name        The associated attribute name
   * @param channel     The pointer to the float channel
   * @return            true if the channel has been loaded successfully, false otherwise
   */
  bool getChannel(const std::string group, const std::string name, lvr2::FloatChannelOptional& channel);

  /**
   * @brief getChannel  Reads an index attribute channel in the given group with the given name
   * @param group       The associated attribute group
   * @param name        The associated attribute name
   * @param channel     The pointer to the index channel
   * @return            true if the channel has been loaded successfully, false otherwise
   */
  bool getChannel(const std::string group, const std::string name, lvr2::IndexChannelOptional& channel);

  /**
   * @brief getChannel  Reads an unsigned char attribute channel in the given group with the given name
   * @param group       The associated attribute group
   * @param name        The associated attribute name
   * @param channel     The pointer to the unsigned char channel
   * @return            true if the channel has been loaded successfully, false otherwise
   */
  bool getChannel(const std::string group, const std::string name, lvr2::UCharChannelOptional& channel);


  /**
   * @brief addChannel  Writes a float attribute channel from the given group with the given name
   * @param group       The associated attribute group
   * @param name        The associated attribute name
   * @param channel     The pointer to the float channel which should be written
   * @return            true if the channel has been written successfully, false otherwise
   */
  bool addChannel(const std::string group, const std::string name, const lvr2::FloatChannel& channel);

  /**
   * @brief addChannel  Writes an index attribute channel from the given group with the given name
   * @param group       The associated attribute group
   * @param name        The associated attribute name
   * @param channel     The pointer to the index channel which should be written
   * @return            true if the channel has been written successfully, false otherwise
   */
  bool addChannel(const std::string group, const std::string name, const lvr2::IndexChannel& channel);

  /**
   * @brief addChannel  Writes an unsigned char attribute channel from the given group with the given name
   * @param group       The associated attribute group
   * @param name        The associated attribute name
   * @param channel     The pointer to the unsigned char channel which should be written
   * @return            true if the channel has been written successfully, false otherwise
   */
  bool addChannel(const std::string group, const std::string name, const lvr2::UCharChannel& channel);

private:

  std::string srv_address;
  std::string srv_endpoint;
  unsigned int srv_port;
  std::array<float, 6> bounding_box;
  httplib::Client http_client;
  Json::Value json_bb;
};


}

#endif // MESH_CLIENT_H_
