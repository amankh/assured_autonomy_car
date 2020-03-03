cc_library(
  name = "units",
  srcs = ["units.cc"],
  hdrs = ["units.h"],
  deps = ["@dreal"],
)

cc_library(
  name = "vectorize",
  srcs = ["vectorize.cc"],
  hdrs = ["vectorize.h"],
  deps = [":units", "@dreal"],
)

cc_library(
  name = "parsing",
  srcs = ["parsing.cc"],
  hdrs = ["parsing.h"],
  deps = ["@dreal"]
)

cc_library(
  name = "layers",
  srcs = ["layers.cc"],
  hdrs = ["layers.h"],
  deps = [":vectorize",
          ":units",
	  ":parsing",
	  "@dreal"],
  copts = ["-std=c++14"],
)

cc_binary(
  name = "circle",
  srcs = ["circle.cc"],
  deps = [":layers"]	,
)

cc_binary(
  name = "straight",
  srcs = ["straight.cc"],
  deps = [":layers"]	,
)

cc_binary(
  name = "test",
  srcs = ["test.cc"],
  deps = [":layers",
       	  "@dreal"],
  copts = ["-std=c++14"],
)