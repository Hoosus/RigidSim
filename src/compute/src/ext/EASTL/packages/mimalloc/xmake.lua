BuildProject({
	projectName = "mimalloc",
	projectType = "shared"
})
add_includedirs("include", {
	public = true
})
add_defines("MI_SHARED_LIB", {
	public = true
})
add_defines("MI_SHARED_LIB_EXPORT", "MI_XMALLOC=1")
add_files("src/static.c")
if is_plat("windows") then
	add_links("psapi", "shell32", "user32", "advapi32", "bcrypt")
	add_defines("_CRT_SECURE_NO_WARNINGS=1", {
		public = true
	})
end
