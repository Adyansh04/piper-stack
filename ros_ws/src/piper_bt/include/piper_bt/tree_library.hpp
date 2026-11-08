#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace piper_bt
{

/**
 * @brief A tree directory as a path: `package://<package>/<path>` lies in that package's share.
 * @param dir A path, or a package URL.
 * @return The directory.
 * @throws ament_index_cpp::PackageNotFoundError If the package is not installed.
 */
[[nodiscard]] std::filesystem::path resolveDirectory(const std::string & dir);

/** @brief A tree's declared inputs: name to default (nullopt: required). */
using TreeInputs = std::map<std::string, std::optional<std::string>>;

/**
 * @brief The inputs each tree of a file declares in its TreeNodesModel.
 * @param xml A BehaviorTree.CPP 4 XML file.
 * @return Tree ID to its input ports and their defaults.
 */
[[nodiscard]] std::map<std::string, TreeInputs> treeInputs(const std::string & xml);

/**
 * @brief The IDs of a file's BehaviorTree elements.
 * @param xml A BehaviorTree.CPP 4 XML file.
 * @return The IDs in file order.
 */
[[nodiscard]] std::vector<std::string> treeIds(const std::string & xml);

/**
 * @class TreeLibrary
 * @brief The trees an executor runs: the XML files of its tree directories, registered in a
 * factory, with each tree's file and declared inputs.
 */
class TreeLibrary
{
public:
    /**
     * @brief The library over tree directories.
     * @param directories Directories (paths or `package://` URLs), searched recursively.
     */
    explicit TreeLibrary(std::vector<std::string> directories);

    /**
     * @brief Registers every file's trees in the factory (node types stay registered).
     * @param factory The executor's factory.
     * @throws std::runtime_error If a file cannot be read or does not load, if two files define one
     * tree ID, or if no tree is found.
     * @throws std::filesystem::filesystem_error If a directory cannot be read.
     */
    void load(BT::BehaviorTreeFactory & factory);

    /** @brief The file each loaded tree comes from. */
    [[nodiscard]] const std::map<std::string, std::filesystem::path> & files() const
    {
        return files_;
    }

    /** @brief Each tree's declared inputs (trees without a model have none listed). */
    [[nodiscard]] const std::map<std::string, TreeInputs> & inputs() const
    {
        return inputs_;
    }

private:
    std::vector<std::string> directories_;
    std::map<std::string, std::filesystem::path> files_;
    std::map<std::string, TreeInputs> inputs_;
};

}  // namespace piper_bt
