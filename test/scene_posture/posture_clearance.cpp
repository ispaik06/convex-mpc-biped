#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Settings {
    std::string xmlPath;
    std::string bodyName = "torso";
    std::string keyName = "nominal_stance";
    std::vector<std::string> siteNames = {"left_foot_contact_site", "right_foot_contact_site"};
};

int findRequiredId(const mjModel* model, const mjtObj type, const char* name) {
    const int id = mj_name2id(model, type, name);
    if (id < 0) {
        throw std::runtime_error(std::string("Could not find MuJoCo object: ") + name);
    }
    return id;
}

double bodyZ(const mjData* data, const int bodyId) {
    return static_cast<double>(data->xpos[3 * bodyId + 2]);
}

double siteZ(const mjData* data, const int siteId) {
    return static_cast<double>(data->site_xpos[3 * siteId + 2]);
}

bool isAbsolutePath(const std::string& path) {
    return !path.empty() && path.front() == '/';
}

std::string resolveRelativeToProjectRoot(const std::string& path) {
    return std::string(PROJECT_ROOT_DIR) + "/" + path;
}

void printUsage(const char* argv0) {
    std::cerr << "usage: " << argv0
              << " --xml <scene.xml> [--body <name>] [--site <name> ...] [--key <key_name>]\n";
    std::cerr << "defaults: body=torso, sites=left_foot_contact_site/right_foot_contact_site, key=nominal_stance\n";
}

Settings parseArgs(int argc, char** argv) {
    Settings settings;
    bool overrideSites = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--xml") {
            if (i + 1 >= argc) {
                printUsage(argv[0]);
                throw std::runtime_error("missing value for --xml");
            }
            settings.xmlPath = argv[++i];
            continue;
        }
        if (arg == "--body") {
            if (i + 1 >= argc) {
                printUsage(argv[0]);
                throw std::runtime_error("missing value for --body");
            }
            settings.bodyName = argv[++i];
            continue;
        }
        if (arg == "--key") {
            if (i + 1 >= argc) {
                printUsage(argv[0]);
                throw std::runtime_error("missing value for --key");
            }
            settings.keyName = argv[++i];
            continue;
        }
        if (arg == "--site") {
            if (i + 1 >= argc) {
                printUsage(argv[0]);
                throw std::runtime_error("missing value for --site");
            }
            if (!overrideSites) {
                settings.siteNames.clear();
                overrideSites = true;
            }
            settings.siteNames.push_back(argv[++i]);
            continue;
        }
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        }

        std::cerr << "unrecognized argument: " << arg << '\n';
        printUsage(argv[0]);
        throw std::runtime_error("unrecognized argument");
    }

    if (settings.xmlPath.empty()) {
        printUsage(argv[0]);
        throw std::runtime_error("missing required --xml");
    }

    return settings;
}

mjModel* loadModel(const std::string& xmlPath, std::string* resolvedPath, std::array<char, 1000>* error) {
    mjModel* model = mj_loadXML(xmlPath.c_str(), nullptr, error->data(), static_cast<int>(error->size()));
    if (model != nullptr) {
        *resolvedPath = xmlPath;
        return model;
    }

    if (!isAbsolutePath(xmlPath)) {
        const std::string projectRootPath = resolveRelativeToProjectRoot(xmlPath);
        model = mj_loadXML(projectRootPath.c_str(), nullptr, error->data(), static_cast<int>(error->size()));
        if (model != nullptr) {
            *resolvedPath = projectRootPath;
            return model;
        }
    }

    *resolvedPath = xmlPath;
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    Settings settings;
    try {
        settings = parseArgs(argc, argv);
    } catch (const std::exception&) {
        return 2;
    }

    std::array<char, 1000> error{};
    std::string resolvedXmlPath;
    mjModel* model = loadModel(settings.xmlPath, &resolvedXmlPath, &error);
    if (model == nullptr) {
        std::cerr << "failed to load " << settings.xmlPath << ": " << error.data() << '\n';
        return 1;
    }

    mjData* data = mj_makeData(model);
    if (data == nullptr) {
        std::cerr << "failed to allocate mjData for " << resolvedXmlPath << '\n';
        mj_deleteModel(model);
        return 1;
    }

    std::string poseSource;
    if (model->nkey == 0) {
        std::cerr << "note: no keyframes found in " << resolvedXmlPath << ", using qpos0\n";
        mj_resetData(model, data);
        poseSource = "qpos0";
    } else {
        const int keyId = mj_name2id(model, mjOBJ_KEY, settings.keyName.c_str());
        if (keyId < 0) {
            std::cerr << "could not find key named '" << settings.keyName << "' in " << resolvedXmlPath << '\n';
            mj_deleteData(data);
            mj_deleteModel(model);
            return 1;
        }
        mj_resetDataKeyframe(model, data, keyId);
        poseSource = std::string("key:") + settings.keyName;
    }
    mj_forward(model, data);

    const int bodyId = findRequiredId(model, mjOBJ_BODY, settings.bodyName.c_str());
    std::vector<int> siteIds;
    siteIds.reserve(settings.siteNames.size());
    for (const std::string& siteName : settings.siteNames) {
        siteIds.push_back(findRequiredId(model, mjOBJ_SITE, siteName.c_str()));
    }

    const double bodyOriginZ = bodyZ(data, bodyId);

    std::vector<double> siteWorldZ;
    std::vector<double> bodyMinusSiteZ;
    siteWorldZ.reserve(siteIds.size());
    bodyMinusSiteZ.reserve(siteIds.size());

    for (const int siteId : siteIds) {
        const double z = siteZ(data, siteId);
        siteWorldZ.push_back(z);
        bodyMinusSiteZ.push_back(bodyOriginZ - z);
    }

    double safeBodyZForAllSites = bodyMinusSiteZ.front();
    for (size_t i = 1; i < bodyMinusSiteZ.size(); ++i) {
        safeBodyZForAllSites = std::max(safeBodyZForAllSites, bodyMinusSiteZ[i]);
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "xml=" << resolvedXmlPath << '\n';
    std::cout << "pose_source=" << poseSource << '\n';
    std::cout << "body_name=" << settings.bodyName << '\n';
    std::cout << "body_origin_z=" << bodyOriginZ << '\n';
    std::cout << "key_name=" << settings.keyName << '\n';
    std::cout << "site_count=" << siteIds.size() << '\n';
    for (size_t i = 0; i < settings.siteNames.size(); ++i) {
        std::cout << "site[" << i << "].name=" << settings.siteNames[i] << '\n';
        std::cout << "site[" << i << "].z=" << siteWorldZ[i] << '\n';
        std::cout << "site[" << i << "].body_minus_site_z=" << bodyMinusSiteZ[i] << '\n';
        std::cout << "site[" << i << "].ground_clearance=" << siteWorldZ[i] << '\n';
    }
    std::cout << "body_z_for_all_sites_on_ground=" << safeBodyZForAllSites << '\n';

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
