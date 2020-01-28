
#ifndef MESH_CLIENT_H_
#define MESH_CLIENT_H_

//  #define CPPHTTPLIB_OPENSSL_SUPPORT

#include <string>
#include <array>
#include <lvr2/io/AttributeMeshIOBase.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/geometry/BaseVector.hpp>
#include <jsoncpp/json/json.h>

namespace mesh_client
{
class MeshClient : public lvr2::AttributeMeshIOBase
{
public:
  /**
   * @brief Constructs a mesh client which receaves
   * @param srv_url The url of the server, use http://localhost/my/path/to/mesh if the server is used locally.
   */
  MeshClient(const std::string& srv_url, const std::string& server_username, const std::string& server_password);

  /**
   * @brief sets the Bounding box for the query which is send to the server
   */
  void setBoundingBox(float min_x, float min_y, float min_z, const float max_x, const float max_y, const float max_z);
  void addFilter(std::string channel, float min_value, float max_value);

  /**
   * @brief Builds a JSON string containing the set bounding
   *        box and the attribute name and the attribute group
   * @param attribute_name The name of the corresponding attribute
   *        to be requested from the server
   * @return the compsoed JSON string following the server's specification
   */
  std::string buildJson(const std::string& attribute_name);

  /**
   * @brief Defines the data type of the transferred channel to decode the byte buffer.
   */
  enum Type : char
  {
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
  std::unique_ptr<std::string> requestChannel(std::string channel);

  std::map<std::string, lvr2::UCharChannel> uchar_channels;
  std::map<std::string, lvr2::IndexChannel> index_channels;
  std::map<std::string, lvr2::FloatChannel> float_channels;

  std::string server_url_;
  std::string server_username_;
  std::string server_password_;

  std::array<float, 6> bounding_box_;
  std::map<std::string, std::pair<float, float>> mesh_filters_;
};

size_t writeFunction(void* ptr, size_t size, size_t nmemb, std::string* data)
{
  data->append((char*)ptr, size * nmemb);
  return size * nmemb;
}

}  // namespace mesh_client

#endif  // MESH_CLIENT_H_
