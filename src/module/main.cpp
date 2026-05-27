#include "discovery.hpp"
#include "realsense.hpp"

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/service.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <boost/thread/synchronized_value.hpp>
#include <librealsense2/rs.hpp>

#if defined(__APPLE__)
#include <unistd.h>
#endif

namespace vsdk = ::viam::sdk;
namespace fs = std::filesystem;

std::vector<std::shared_ptr<vsdk::ModelRegistration>>
create_all_model_registrations(
    std::shared_ptr<
        realsense::RealsenseContext<boost::synchronized_value<rs2::context>>>
        realsense_ctx,
    std::shared_ptr<boost::synchronized_value<std::unordered_set<std::string>>>
        assigned_serials) {
  std::vector<std::shared_ptr<vsdk::ModelRegistration>> registrations;

  registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
      vsdk::API::get<vsdk::Camera>(),
      realsense::Realsense<boost::synchronized_value<rs2::context>>::model,
      [realsense_ctx, assigned_serials](vsdk::Dependencies deps,
                                        vsdk::ResourceConfig config) {
        return std::make_unique<
            realsense::Realsense<boost::synchronized_value<rs2::context>>>(
            std::move(deps), std::move(config), realsense_ctx,
            assigned_serials);
      },
      realsense::Realsense<realsense::RealsenseContext<
          boost::synchronized_value<rs2::context>>>::validate));

  registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
      vsdk::API::get<vsdk::Discovery>(),
      realsense::discovery::RealsenseDiscovery<
          boost::synchronized_value<rs2::context>>::model,
      [realsense_ctx](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
        return std::make_unique<realsense::discovery::RealsenseDiscovery<
            realsense::RealsenseContext<
                boost::synchronized_value<rs2::context>>>>(
            std::move(deps), std::move(config), realsense_ctx);
      }));

  return registrations;
}

int serve(int argc, char **argv) try {
  // Every Viam C++ SDK program must have one and only one Instance object
  // which is created before any other C++ SDK objects and stays alive until
  // all Viam C++ SDK objects are destroyed.
  vsdk::Instance inst;

  VIAM_SDK_LOG(info) << "[serve] Starting Realsense module";

#if defined(__APPLE__)
  // Log user ID and fail if it is Apple and not root
  auto uid = getuid();
  if (uid != 0) {
    std::cerr << "[serve] Realsense module is not running as root (user ID = "
              << uid
              << "), initialize viam-server with sudo: sudo viam-server "
                 "-config <path_to_config>"
              << std::endl;
    return EXIT_FAILURE;
  }

  // Enabling zlibrealsense debug logs for Mac only for now, as we don't count
  // with libraries with BUILD_EASYLOGGINGPP enabled on Linux, which is required
  // to enable debug logs.
  // https://viam.atlassian.net/browse/RSDK-13059
  for (size_t i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--log-level=debug") {
      rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    }
  }
#endif

  auto ctx = std::make_shared<boost::synchronized_value<rs2::context>>();
  // Wrap the context in a RealsenseContext, which will manage the callback for
  // device changes and notify all Realsense instances.
  // It also provides a thread-safe way to query connected devices.
  auto rs_ctx = std::make_shared<
      realsense::RealsenseContext<boost::synchronized_value<rs2::context>>>(
      ctx);

  // This keeps track of serial numbers that have already been assigned to
  // Realsense instances, to avoid assigning the same physical camera to
  // multiple instances.
  auto assigned_serials = std::make_shared<
      boost::synchronized_value<std::unordered_set<std::string>>>();

  auto module_service = std::make_shared<vsdk::ModuleService>(
      argc, argv, create_all_model_registrations(rs_ctx, assigned_serials));
  module_service->serve();

  return EXIT_SUCCESS;
} catch (const std::exception &ex) {
  std::cerr << "ERROR: A std::exception was thrown from `serve`: " << ex.what()
            << std::endl;
  return EXIT_FAILURE;
} catch (...) {
  std::cerr << "ERROR: An unknown exception was thrown from `serve`"
            << std::endl;
  return EXIT_FAILURE;
}

namespace {

void writeBytesToFile(fs::path const &path,
                      std::vector<unsigned char> const &bytes) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    throw std::runtime_error("failed to open " + path.string() +
                             " for writing");
  }
  out.write(reinterpret_cast<const char *>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
  if (!out) {
    throw std::runtime_error("failed to write to " + path.string());
  }
}

bool hasFlag(int argc, char **argv, std::string const &flag) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == flag) {
      return true;
    }
  }
  return false;
}

std::optional<std::string> flagValue(int argc, char **argv,
                                     std::string const &flag) {
  std::string with_eq = flag + "=";
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a.rfind(with_eq, 0) == 0) {
      return a.substr(with_eq.size());
    }
    if (a == flag && i + 1 < argc) {
      return std::string(argv[i + 1]);
    }
  }
  return std::nullopt;
}

} // namespace

// Standalone mode: open a RealSense device, capture one frameset, write
// color JPEG + depth + PCD to disk, and exit. Bypasses ModuleService so the
// module can be exercised without a viam-server parent process.
int runStandalone(int argc, char **argv) try {
  fs::path out_dir = fs::current_path();
  // First non-flag arg after --standalone is the output directory.
  for (int i = 2; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg.rfind("--", 0) != 0) {
      out_dir = arg;
      break;
    }
  }
  if (!fs::exists(out_dir)) {
    fs::create_directories(out_dir);
  }

  // Default to 640x480: the macOS librealsense UVC backend handles
  // lower-bandwidth isochronous endpoint configs reliably, whereas higher
  // resolutions (e.g. 1280x720@30) are intermittent — pipe.start sometimes
  // fails with "failed to set power state", and sometimes succeeds but the
  // streaming-start commands silently no-op so no frames arrive. Override
  // with --width/--height to reproduce/test those higher-res paths.
  int width = 640;
  int height = 480;
  int timeout_sec = 10;
  int count = 1;
  double delay_sec = 1.0;
  if (auto v = flagValue(argc, argv, "--width")) {
    width = std::stoi(*v);
  }
  if (auto v = flagValue(argc, argv, "--height")) {
    height = std::stoi(*v);
  }
  if (auto v = flagValue(argc, argv, "--timeout")) {
    timeout_sec = std::stoi(*v);
  }
  if (auto v = flagValue(argc, argv, "--count")) {
    count = std::stoi(*v);
  }
  if (auto v = flagValue(argc, argv, "--delay")) {
    delay_sec = std::stod(*v);
  }
  if (count < 1) {
    count = 1;
  }
  if (delay_sec < 0.0) {
    delay_sec = 0.0;
  }
  const auto delay_ms =
      std::chrono::milliseconds(static_cast<long>(delay_sec * 1000.0));

  std::cout << "[standalone] output directory: " << out_dir << std::endl;
  std::cout << "[standalone] requesting " << width << "x" << height
            << " (color+depth), timeout " << timeout_sec << "s, " << count
            << " capture(s), " << delay_sec << "s between captures"
            << std::endl;

  vsdk::Instance inst;

#if defined(__APPLE__)
  if (geteuid() != 0) {
    std::cerr << "[standalone] must be run as root on macOS — the UVC kernel "
                 "driver claims USB interface 0 and only root can take it back. "
                 "Re-run with: sudo "
              << argv[0] << " --standalone " << out_dir.string() << std::endl;
    return EXIT_FAILURE;
  }
#endif

  if (hasFlag(argc, argv, "--log-level=debug")) {
    rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
  }

  auto ctx = std::make_shared<boost::synchronized_value<rs2::context>>();
  auto rs_ctx = std::make_shared<
      realsense::RealsenseContext<boost::synchronized_value<rs2::context>>>(
      ctx);
  auto assigned_serials = std::make_shared<
      boost::synchronized_value<std::unordered_set<std::string>>>();

  vsdk::ProtoStruct attributes;
  vsdk::ProtoList sensors_list = {std::string("color"), std::string("depth")};
  attributes["sensors"] = sensors_list;
  attributes["width_px"] = static_cast<double>(width);
  attributes["height_px"] = static_cast<double>(height);

  vsdk::ResourceConfig cfg(
      "rdk:component:camera", "", "standalone", attributes, "",
      vsdk::Model("viam", "camera", "realsense"), vsdk::LinkConfig{},
      vsdk::log_level::info);

  std::cout << "[standalone] constructing camera (this opens the device)..."
            << std::endl;
  realsense::Realsense<boost::synchronized_value<rs2::context>> camera(
      vsdk::Dependencies{}, std::move(cfg), rs_ctx, assigned_serials);
  std::cout << "[standalone] camera constructed; starting capture loop..."
            << std::endl;

  // The pipeline runs in a librealsense-owned thread and calls our frame
  // callback asynchronously. Poll get_images() until it returns a frameset, or
  // until we time out.
  auto poll_images = [&]() -> vsdk::Camera::image_collection {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
    vsdk::Camera::image_collection imgs;
    std::string last_err;
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        imgs = camera.get_images({}, {});
        if (!imgs.images.empty()) {
          return imgs;
        }
      } catch (const std::exception &e) {
        last_err = e.what();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!last_err.empty()) {
      std::cerr << "[standalone] last get_images error: " << last_err
                << std::endl;
    }
    return imgs; // empty
  };

  // When capturing more than one image, suffix filenames with the capture
  // index so they don't overwrite each other (color_000.jpg, depth_000.viam).
  auto indexed_name = [count](std::string const &stem, std::string const &ext,
                              int idx) {
    if (count <= 1) {
      return stem + ext;
    }
    std::ostringstream oss;
    oss << stem << "_" << std::setfill('0') << std::setw(3) << idx << ext;
    return oss.str();
  };

  int captured = 0;
  for (int n = 0; n < count; ++n) {
    auto imgs = poll_images();
    if (imgs.images.empty()) {
      std::cerr << "[standalone] capture " << (n + 1) << "/" << count
                << ": timed out waiting for frames" << std::endl;
    } else {
      for (auto const &img : imgs.images) {
        std::string filename;
        if (img.source_name == "color") {
          filename = indexed_name("color", ".jpg", n);
        } else if (img.source_name == "depth") {
          filename = indexed_name("depth", ".viam", n);
        } else {
          filename = indexed_name(img.source_name, ".bin", n);
        }
        auto path = out_dir / filename;
        writeBytesToFile(path, img.bytes);
        std::cout << "[standalone] capture " << (n + 1) << "/" << count
                  << ": wrote " << path << " (" << img.bytes.size()
                  << " bytes, mime=" << img.mime_type
                  << ", source=" << img.source_name << ")" << std::endl;
      }
      ++captured;
    }

    if (n + 1 < count && delay_ms.count() > 0) {
      std::this_thread::sleep_for(delay_ms);
    }
  }

  if (captured == 0) {
    std::cerr << "[standalone] no images captured" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "[standalone] done (" << captured << "/" << count
            << " captures succeeded)" << std::endl;
  return EXIT_SUCCESS;
} catch (const rs2::error &e) {
  std::cerr << "[standalone] librealsense error in "
            << e.get_failed_function() << "(" << e.get_failed_args()
            << "): " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception &e) {
  std::cerr << "[standalone] error: " << e.what() << std::endl;
  return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {
  std::cout << "Realsense C++ SDK version: " << RS2_API_VERSION_STR << "\n";

  if (argc >= 2 && std::string(argv[1]) == "--standalone") {
    return runStandalone(argc, argv);
  }

  const std::string usage =
      "usage: realsense /path/to/unix/socket\n"
      "       realsense --standalone [out_dir] [--width N] [--height N] "
      "[--count N] [--delay SECONDS] [--timeout SECONDS] [--log-level=debug]";
  if (argc < 2) {
    std::cout << "ERROR: insufficient arguments\n";
    std::cout << usage << "\n";
    return EXIT_FAILURE;
  }

  return serve(argc, argv);
}
