# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  # All Vagrant configuration is done here. The most common configuration
  # options are documented and commented below. For a complete reference,
  # please see the online documentation at vagrantup.com.

  # Every Vagrant virtual environment requires a box to build off of.
  config.vm.box = "ubuntu/xenial64"

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
    # Enable usb
    vb.customize ['modifyvm', :id, "--usb", "on"]
	# vb.customize ["modifyvm", :id, "--usbehci", "on"] # requires extention

	# To get list of usb devices use "VBoxManage list usbhost" and copy the
    # address for the desired device

	# auto attach the following USB devices
	vb.customize ["usbfilter", "add", "0", "--target", :id,
      "--name", "STM32 STLink",
      "--action", "hold", # Makes the device available

      #"--manufacturer", "STMicroelectronics",

      "--vendorid", "0x0483",
      "--productid", "0x3748",
      #"--product", "STM32 STLink",
      # "--serialnumber", "A600JE0S"
    ]
  end

  # Install arm-none-eabi toolchain
  config.vm.provision "shell",
    inline: "apt-get -y update && apt-get -y install gcc-arm-none-eabi libnewlib-arm-none-eabi cmake git openocd"

  # configure and make the project so the user is ready to work affter ssh
  config.vm.provision "shell",
    inline: "cmake /vagrant && make", privileged: false
end
