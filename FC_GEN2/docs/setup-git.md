# Git SSH Setup

This guide is to create an SSH key to be connected to a GitHub profile in order to clone the project using SSH.

## Setting up an SSH key using ssh-keygen

Navigate to the `~/.ssh/` directory on your machine and enter the command `ssh-keygen`.
Press enter to skip each prompt and use the default settings.

## Connecting your SSH key with your GitHub profile

Copy the contents of the newly generated `id_rsa.pub` file and navigate to the settings of your GitHub account. In the SSH and GPG keys tab, click New SSH key and add the string that you copied.

## Cloning the project using SSH

```bash
git clone git@github.com:macformula/front_controller.git
```