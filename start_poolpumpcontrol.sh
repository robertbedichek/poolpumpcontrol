#!/bin/zsh

sudo launchctl bootstrap gui/$(id -u) $HOME/Library/LaunchAgents/com.bedichek.watch_gobble_poolpumpcontrol.plist	
ps aux|egrep poolpumpcontrol
