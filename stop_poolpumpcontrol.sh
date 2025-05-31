#!/bin/zsh

sudo launchctl bootout gui/$(id -u) $HOME/Library/LaunchAgents/com.bedichek.watch_gobble_poolpumpcontrol.plist
ps aux|egrep poolpumpcontrol
