#!/bin/zsh

sudo launchctl bootout gui/$(id -u) /Users/robertbedichek/Library/LaunchAgents/com.bedichek.watch_gobble_poolpumpcontrol.plist
ps aux|egrep gobble
