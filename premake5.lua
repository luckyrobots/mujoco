project "tinyxml2-mujoco"
	kind "StaticLib"
	language "C++"
	cppdialect "C++17"
    staticruntime "off"

	targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

	files
	{
		"src/vendor/tinyxml2/tinyxml2.cpp",
		"src/vendor/tinyxml2/tinyxml2.h"
	}

	includedirs
	{
		"src/vendor/tinyxml2"
	}

	filter "system:windows"
		systemversion "latest"

	filter "configurations:Debug"
		runtime "Debug"
		symbols "on"

	filter "configurations:Release"
		runtime "Release"
		optimize "on"

	filter "configurations:Dist"
		runtime "Release"
		optimize "on"
        symbols "off"

project "mujoco"
	kind "StaticLib"
	language "C++"
	cppdialect "C++20"

	targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

	staticruntime "off"
	warnings "off"

	-- For ccd, create config.h if doesn't already exist (to match cmake)
	local configHeader = "src/vendor/ccd/src/ccd/config.h"
	if not os.isfile(configHeader) then
        print("Generating " .. configHeader)
		io.writefile(configHeader, [[
#ifndef __CCD_CONFIG_H__
#define __CCD_CONFIG_H__

#define CCD_DOUBLE

#endif /* __CCD_CONFIG_H__ */
        ]])
    end

	files
	{
		"include/mujoco/*.h",

		"src/engine/*.h",
		"src/engine/*.c",
		"src/engine/*.cc",

		"src/xml/*.h",
		"src/xml/*.cc",

		"src/user/*.h",
		"src/user/*.c",
		"src/user/*.cc",

		"src/thread/*.h",
		"src/thread/*.cc",

		-- Plugins (first-party)
		"plugin/sdf/*.h",
		"plugin/sdf/*.cc",

		-- Dependencies (compiled here for now)
		"src/vendor/ccd/src/*.h",
		"src/vendor/ccd/src/*.c",

		-- Outlined tinyxml2 to Dependencies.lua
		"src/vendor/tinyobjloader/tiny_obj_loader.h",
		"src/vendor/tinyobjloader/tiny_obj_loader.cc",

		"src/vendor/lodepng/lodepng.h",
		"src/vendor/lodepng/lodepng.cpp",

		"src/vendor/qhull/src/libqhull_r/**.h",
		"src/vendor/qhull/src/libqhull_r/**.c",
	}

	includedirs
	{
		"include",
		"src",

		"src/vendor/ccd/src",
		"src/vendor/tinyxml2",
		"src/vendor/tinyobjloader",
		"src/vendor/marchingcubecpp",
		"src/vendor/trianglemeshdistance",
		"src/vendor/lodepng",
		"src/vendor/qhull/src",
		"src/vendor/qhull/src/libqhull_r",
	}

	defines
	{
		"MJ_STATIC",
		"MC_IMPLEM_ENABLE"
	}

	filter "configurations:Debug"
		runtime "Debug"
		symbols "on"

	filter { "system:windows", "configurations:Debug-AS" }	
		runtime "Debug"
		symbols "on"
		sanitize { "Address" }
		flags { "NoRuntimeChecks", "NoIncrementalLink" }

	filter "configurations:Release"
		runtime "Release"
		optimize "speed"

    filter "configurations:Dist"
		runtime "Release"
		optimize "speed"
        symbols "off"

	-- Needed for `strnlen`.
	filter "toolset:msc"
		cdialect "C11"
	filter "toolset:gcc or toolset:clang"	
		cdialect "gnu11"

