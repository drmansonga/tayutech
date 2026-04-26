variable "raspberry_devices" {
  type = map(object({
    hostname     = string
    ip           = string
    user         = string
    model        = string      # "pi5" or "pico2w"
    role         = string      # "server" or "sensor"
    architecture = string      # "arm64" or "armv6l" or "none"
  }))
  default = {
    pi5-01 = {
      hostname     = "iot-server-01"
      ip           = "192.168.1.101"
      user         = "pi"
      model        = "pi5"
      role         = "server"
      architecture = "arm64"
    }
    pi5-02 = {
      hostname     = "iot-server-02"
      ip           = "192.168.1.102"
      user         = "pi"
      model        = "pi5"
      role         = "server"
      architecture = "arm64"
    }
    pico-01 = {
      hostname     = "iot-sensor-01"
      ip           = "192.168.1.201"
      user         = "pi"           # or whatever user you set
      model        = "pico2w"
      role         = "sensor"
      architecture = "none"         # no full OS
    }
    # Add more devices here
  }
}

variable "ssh_private_key_path" {
  type    = string
  default = "~/.ssh/raspberry-iot"
}







//variable "ssh_private_key_path" {
  //default = "~/.ssh/raspberry-iot"   # or the exact path you used

