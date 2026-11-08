#include "piper_bt/tree_library.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_path.hpp>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace piper_bt
{

namespace
{

std::string readFile(const std::filesystem::path & file)
{
    std::ifstream in(file);
    if (!in)
    {
        throw std::runtime_error(std::format("tree file {} cannot be read", file.string()));
    }
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

std::filesystem::path resolveDirectory(const std::string & dir)
{
    constexpr std::string_view kScheme = "package://";
    if (!dir.starts_with(kScheme))
    {
        return dir;
    }
    const std::string rest = dir.substr(kScheme.size());
    const auto slash = rest.find('/');
    const std::string package = rest.substr(0, slash);
    const std::filesystem::path share = ament_index_cpp::get_package_share_path(package);
    return slash == std::string::npos ? share : share / rest.substr(slash + 1);
}

std::vector<std::string> treeIds(const std::string & xml)
{
    static const std::regex kTree(R"re(<BehaviorTree\b[^>]*\bID\s*=\s*"([^"]+)")re");
    std::vector<std::string> ids;
    for (auto it = std::sregex_iterator(xml.begin(), xml.end(), kTree);
         it != std::sregex_iterator();
         ++it)
    {
        ids.push_back((*it)[1].str());
    }
    return ids;
}

std::map<std::string, TreeInputs> treeInputs(const std::string & xml)
{
    static const std::regex kModel(
        R"re(<SubTree\b[^>]*\bID\s*=\s*"([^"]+)"[^>]*>([\s\S]*?)</SubTree>)re");
    static const std::regex kPort(R"re(<input_port\b([^>]*)>)re");
    static const std::regex kName(R"re(\bname\s*=\s*"([^"]*)")re");
    static const std::regex kDefault(R"re(\bdefault\s*=\s*"([^"]*)")re");
    std::map<std::string, TreeInputs> out;
    const auto model_start = xml.find("<TreeNodesModel");
    if (model_start == std::string::npos)
    {
        return out;
    }
    const std::string models = xml.substr(model_start);
    for (auto m = std::sregex_iterator(models.begin(), models.end(), kModel);
         m != std::sregex_iterator();
         ++m)
    {
        TreeInputs & inputs = out[(*m)[1].str()];
        const std::string body = (*m)[2].str();
        for (auto p = std::sregex_iterator(body.begin(), body.end(), kPort);
             p != std::sregex_iterator();
             ++p)
        {
            const std::string attributes = (*p)[1].str();
            std::smatch name;
            if (!std::regex_search(attributes, name, kName))
            {
                continue;
            }
            std::smatch fallback;
            inputs[name[1].str()] = std::regex_search(attributes, fallback, kDefault)
                                        ? std::optional<std::string>(fallback[1].str())
                                        : std::nullopt;
        }
    }
    return out;
}

TreeLibrary::TreeLibrary(std::vector<std::string> directories)
    : directories_(std::move(directories))
{}

void TreeLibrary::load(BT::BehaviorTreeFactory & factory)
{
    std::vector<std::filesystem::path> files;
    for (const std::string & dir : directories_)
    {
        for (const auto & entry :
             std::filesystem::recursive_directory_iterator(resolveDirectory(dir)))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".xml")
            {
                files.push_back(std::filesystem::weakly_canonical(entry.path()));
            }
        }
    }
    std::ranges::sort(files);
    const auto [last, end] = std::ranges::unique(files);
    files.erase(last, end);
    for (const auto & file : files)
    {
        const std::string xml = readFile(file);
        const std::vector<std::string> ids = treeIds(xml);
        // A later file with an ID already loaded would replace that tree without a word.
        for (const std::string & id : ids)
        {
            if (const auto seen = files_.find(id); seen != files_.end())
            {
                throw std::runtime_error(
                    std::format(
                        "tree {} is in both {} and {}", id, seen->second.string(), file.string()));
            }
        }
        try
        {
            factory.registerBehaviorTreeFromText(xml);
        }
        catch (const std::exception & e)
        {
            throw std::runtime_error(
                std::format("tree file {} does not load: {}", file.string(), e.what()));
        }
        for (const std::string & id : ids)
        {
            files_.emplace(id, file);
        }
        inputs_.merge(treeInputs(xml));
    }
    if (files_.empty())
    {
        throw std::runtime_error("no behaviour tree in the tree directories");
    }
}

}  // namespace piper_bt
