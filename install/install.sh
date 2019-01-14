#adduser --system --group --no-create-home spaxx
systemctl stop spaxxserver
mkdir -p /var/lib/spaxx/
cp spaxxserver.ini /var/lib/spaxx/spaxxserver.ini
#chown spaxx:spaxx /var/lib/spaxx
#chmod 770 /var/lib/spaxx
#chmod 770 /var/lib/spaxx/*

LOGFOLDER=/var/log/spaxx
mkdir -p $LOGFOLDER
#chown spaxx:spaxx $LOGFOLDER
#chmod 776 $LOGFOLDER

cp spaxxserver /usr/sbin/spaxxserver
cp install/spaxxserver.service /etc/systemd/system/spaxxserver.service 
# Start spaxxserver

systemctl daemon-reload
systemctl start spaxxserver

# Enable Redis to Start at Boot
systemctl enable spaxxserver