# SSH instruction

To run ROS on SSH and Webots locally, it is needed to forward requests to the local machine where Webots server is listening on port 2000, and also Webots messages on port 1234. Additionally, also a shared folder is used by Webots to share world files, as specified by env variable `WEBOTS_SHARED_FOLDER=/Users/Shared/shared:/root/shared` on DockerRunEnv.

## Remotely

- Start forwarding services using script `start.sh`

## Locally

- Mount Webots shared folder

    > On macOS need to setup FUSE and SSHFS https://susanqq.github.io/jekyll/pixyll/2017/09/05/remotefiles/
    
    ```console
    sudo sshfs -o allow_other,defer_permissions,IdentityFile=~/.ssh/id_rsa user@host:/Users/Shared/shared /Users/Shared/shared
    ```

    Check what is mounted using command `mount`

- To unmount
    ```console
    sudo umount user@host:/Users/Shared/shared
    ```

