#!/bin/bash

# Grab the current directory
directory=$(pwd)
echo "Using Directory: "$directory

# Generate the run file
infile=${directory}/armstron/bash/run_armstron_template.sh
sed "s#DIRECTORY#$directory#" $infile > run_armstron.sh
echo 'Generated "run_armstron.sh"'

# Generate the .desktop file
infile=${directory}/armstron/bash/armstron_template.desktop
sed "s#DIRECTORY#$directory#" $infile > Armstron.desktop
echo 'Generated "Armstron.desktop"' 

# Make a link to the .desktop file so it shows up in Applications
rm /usr/share/applications/Armstron.desktop 2> /dev/null
ln -s $directory/Armstron.desktop /usr/share/applications/Armstron.desktop
echo 'Created symbolic link for Armstron.desktop file in "/usr/share/applications"'