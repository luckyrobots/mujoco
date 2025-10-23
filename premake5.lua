project "mujoco"
	kind "StaticLib"
	language "C++"
	cppdialect "C++20"
	cdialect "C11"

	staticruntime "off"
	warnings "off"

	targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

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

		-- Dependencies (compiled here for now)
		"src/vendor/ccd/src/*.h",
		"src/vendor/ccd/src/*.c",

		"src/vendor/tinyxml2/tinyxml2.h",
		"src/vendor/tinyxml2/tinyxml2.cpp",
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
