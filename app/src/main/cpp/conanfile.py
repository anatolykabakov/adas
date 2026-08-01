from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain


class AdasCppConan(ConanFile):
    name = "adas-cpp"
    version = "0.1.0"
    package_type = "application"
    settings = "os", "arch", "compiler", "build_type"
    options = {"tests": [True, False], "python_bindings": [True, False]}
    default_options = {
        "tests": False,
        "python_bindings": False,
        "zeromq/*:encryption": False,
        "libusb/*:enable_udev": False,
    }

    def requirements(self):
        self.requires("protobuf/3.21.12")
        self.requires("cppzmq/4.10.0")
        self.requires("libusb/1.0.26")
        self.requires("jsoncpp/1.9.6")
        self.requires("eigen/3.4.0")
        if self.options.tests:
            self.requires("gtest/1.14.0")
        if self.options.python_bindings:
            self.requires("pybind11/2.11.1")

    def build_requirements(self):
        self.tool_requires("protobuf/3.21.12")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["BUILD_FOR_ANDROID"] = self.settings.os == "Android"
        tc.variables["BUILD_TESTING"] = bool(self.options.tests)
        tc.variables["BUILD_PYTHON_BINDINGS"] = bool(self.options.python_bindings)
        tc.generate()
        CMakeDeps(self).generate()
