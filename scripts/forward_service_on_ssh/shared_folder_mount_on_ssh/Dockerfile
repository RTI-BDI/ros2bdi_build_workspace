FROM sesamcommunity/sshfs

RUN apk update && apk upgrade && apk add sshfs

COPY ~/.ssh ~/.ssh

RUN echo "user_allow_other" | tee -a /etc/fuse.conf
RUN mkdir -p /mnt/shared
RUN chmod 600 /home/files/key.pem

CMD sh -c "sudo sshfs -o allow_other,defer_permissions,IdentityFile=~/.ssh/id_rsa user@host:/Users/Shared/shared /root/shared"


# build
# sudo docker build --rm --tag sshfs .

# run
# docker run -v ~/.ssh:/root/.ssh -v /Users/Shared/shared:/root/shared -e "USER=marco" -e "HOST=rtibdi.disi.unitn.it" --rm -it sshfs

