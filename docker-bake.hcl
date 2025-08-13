group "default" {
  targets = ["novnc", "optitrack", "server-dev"]
}

// target "novnc" {
//   context    = "./docker/novnc"
//   dockerfile = "Dockerfile"
//   tags       = ["tabletop/novnc:latest"]
// }

// target "server-base" {
//   context    = "."
//   dockerfile = "docker/server/Dockerfile"
//   args = {
//     ROS_DISTRO = "jazzy"
//     UID        = "1000"
//     USER       = "mules"
//   }
//   tags   = ["tabletop/server-base:latest"]
//   target = "base"
// }

// target "server-dev" {
//   context    = "."
//   dockerfile = "docker/server/Dockerfile"
//   args = {
//     ROS_DISTRO = "jazzy"
//     UID        = "1000"
//     USER       = "mules"
//   }
//   tags   = ["tabletop/server-dev:latest"]
//   target = "dev"
// }
