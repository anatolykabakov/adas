"""Рецепт acados: качает исходники и собирает их нашим тулчейном.

Версия 0.1.8 обязательна: сгенерированный решатель в src/lateral/acados_lat_ocp взят у
dragonpilot и совместим только с ней (в v0.1.9 переименован ocp_nlp_dynamics_dims_get_from_attr).

Зеркало по умолчанию — нексус, github идёт вторым адресом:
    conan install ... -c user.acados:mirror=https://github.com
"""

import os

from conan import ConanFile
from conan.errors import ConanInvalidConfiguration
from conan.tools.cmake import CMake, CMakeToolchain, cmake_layout
from conan.tools.files import copy, get, rmdir

SUBMODULES = {
    "blasfeo": ("giaf/blasfeo", "edf92b396adddd9e548b9786f87ad290a0971329",
                "84825c30e0a68eca6da8d4009f157f4278ef0f607347af853eb49a331b67ac0d"),
    "hpipm": ("giaf/hpipm", "7f8e8de828be86b6ae3d373e8e56f9f3b1fd45aa",
              "8a19f998fbcf4d5ec68dbcc9e42bfe668feef873bb26e094990c766e4b54f505"),
    "qpoases": ("acados/qpOASES", "77ce8944825d6857c606655ed60ced5ea30017a5",
                "87b3df618d2adcba89feabc44b30234805389a6e10da7495149f209e6d4e1bca"),
}
ACADOS_SHA256 = "aa43680f9dc626a77bfc56664f1454cb1926464499b6fb02ac780533c438525d"
GITHUB = "https://github.com"
DEFAULT_MIRROR = "https://github.com"


class AcadosConan(ConanFile):
    name = "acados"
    version = "0.1.8"
    license = "BSD-2-Clause"
    homepage = "https://github.com/acados/acados"
    description = "Fast embedded solvers for nonlinear optimal control"
    topics = ("mpc", "optimal-control", "hpipm", "blasfeo")

    package_type = "shared-library"
    settings = "os", "arch", "compiler", "build_type"
    options = {
        "qpoases": [True, False],
        "blasfeo_target": ["ANY"],
        "hpipm_target": ["ANY"],
        "openmp": [True, False],
    }
    default_options = {
        "qpoases": False,
        "blasfeo_target": "auto",
        "hpipm_target": "auto",
        "openmp": False,
    }

    @property
    def _mirror(self):
        return self.conf.get("user.acados:mirror", default=DEFAULT_MIRROR, check_type=str).rstrip("/")

    def _auto_target(self):
        if self.settings.arch == "armv8":
            return "ARMV8A_ARM_CORTEX_A57"
        if self.settings.arch == "x86_64":
            return "X64_AUTOMATIC"
        return "GENERIC"

    def validate(self):
        if self.settings.os not in ("Linux", "Android"):
            raise ConanInvalidConfiguration(f"поддерживаются Linux и Android, не {self.settings.os}")

    def layout(self):
        cmake_layout(self, src_folder="src")

    def _urls(self, path):
        urls = [f"{self._mirror}/{path}"]
        if not self._mirror.startswith(GITHUB):
            urls.append(f"{GITHUB}/{path}")
        return urls

    def source(self):
        get(self, self._urls(f"acados/acados/archive/refs/tags/v{self.version}.tar.gz"),
            sha256=ACADOS_SHA256, strip_root=True)
        for name, (repo, commit, sha) in SUBMODULES.items():
            get(self, self._urls(f"{repo}/archive/{commit}.tar.gz"), sha256=sha, strip_root=True,
                destination=os.path.join(self.source_folder, "external", name))

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["ACADOS_INSTALL_DIR"] = self.package_folder.replace("\\", "/")
        tc.cache_variables["BUILD_SHARED_LIBS"] = True
        tc.cache_variables["ACADOS_EXAMPLES"] = False
        tc.cache_variables["ACADOS_UNIT_TESTS"] = False
        tc.cache_variables["ACADOS_LINT"] = False
        tc.cache_variables["ACADOS_PYTHON"] = False
        tc.cache_variables["ACADOS_MATLAB"] = False
        tc.cache_variables["ACADOS_OCTAVE"] = False
        tc.cache_variables["ACADOS_WITH_OPENMP"] = bool(self.options.openmp)
        tc.cache_variables["ACADOS_WITH_QPOASES"] = bool(self.options.qpoases)
        tc.cache_variables["BLASFEO_EXAMPLES"] = False
        tc.cache_variables["BLASFEO_TESTING"] = False
        bt = str(self.options.blasfeo_target)
        ht = str(self.options.hpipm_target)
        tc.cache_variables["BLASFEO_TARGET"] = self._auto_target() if bt == "auto" else bt
        tc.cache_variables["HPIPM_TARGET"] = self._auto_target() if ht == "auto" else ht
        if self.settings.os == "Android":
            tc.cache_variables["BLASFEO_CROSSCOMPILING"] = True
            tc.preprocessor_definitions["OS_LINUX"] = None
            tc.cache_variables["CMAKE_ASM_FLAGS"] = "-DOS_LINUX"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        CMake(self).install()
        copy(self, "LICENSE", self.source_folder, os.path.join(self.package_folder, "licenses"))
        for stale in ("cmake", "lib/cmake"):
            rmdir(self, os.path.join(self.package_folder, stale))

    def package_info(self):
        libs = ["acados", "hpipm", "blasfeo"]
        if self.options.qpoases:
            libs.insert(1, "qpOASES_e")
        self.cpp_info.libs = libs
        self.cpp_info.includedirs = ["include", "include/blasfeo/include", "include/hpipm/include"]
        if self.settings.os in ("Linux", "Android"):
            self.cpp_info.system_libs = ["m"]
        self.cpp_info.set_property("cmake_file_name", "acados")
        self.cpp_info.set_property("cmake_target_name", "acados::acados")
