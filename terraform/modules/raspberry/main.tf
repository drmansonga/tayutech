resource "null_resource" "bootstrap" {
  for_each = var.raspberry_devices

  triggers = {
    device_id = each.key
  }

  connection {
    type        = "ssh"
    user        = each.value.user
    host        = each.value.ip
    private_key = file(var.ssh_private_key_path)
  }

  # Only run full bootstrap on Pi 5 / server devices
  provisioner "remote-exec" {
    when = each.value.role == "server" && each.value.model == "pi5"

    inline = [
      "sudo apt-get update && sudo apt-get upgrade -y",
      "sudo apt-get install -y ansible python3-pip git docker.io docker-compose-plugin",
      "sudo usermod -aG docker ${each.value.user}",
      "sudo systemctl enable --now docker",
      "echo 'Device ${each.key} (${each.value.model}) bootstrapped as server'"
    ]
  }

  # Minimal bootstrap for Pico 2 W (if it runs any Linux)
  provisioner "remote-exec" {
    when = each.value.model == "pico2w"

    inline = [
      "echo 'Pico 2 W detected - minimal setup only (no Docker)'",
      "sudo apt-get update && sudo apt-get install -y python3-pip git",
      "echo 'Pico 2 W ready for MicroPython/C SDK flashing or basic MQTT client'"
    ]
  }
}

resource "null_resource" "deploy_stack" {
  depends_on = [null_resource.bootstrap]

  for_each = { for k, v in var.raspberry_devices : k => v if v.role == "server" && v.model == "pi5" }

  provisioner "local-exec" {
    command = <<EOT
      ansible-playbook -i ../../ansible/inventory.ini \
        -u ${each.value.user} \
        --private-key ${var.ssh_private_key_path} \
        --extra-vars "target_host=${each.value.ip} hostname=${each.value.hostname} model=${each.value.model}" \
        ../../ansible/playbooks/stack.yml
    EOT
  }
}
