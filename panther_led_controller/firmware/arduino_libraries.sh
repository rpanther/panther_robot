 #!/bin/bash
# Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

red=`tput setaf 1`
green=`tput setaf 2`
reset=`tput sgr0`

# Check if a path is included
if [ -z $1 ] ; then
    echo "${red}Require to pass a path!${reset}"
    exit 1
fi
# Get path
LIBRARY_PATH="$1/libraries"
# Library to download
AUTHOR=$2
REPOSITORY=$3
# Get latest release from GitHub api, Get tag line
echo "Find last version of $AUTHOR $REPOSITORY"
TAG_VERSION=$(curl --silent "https://api.github.com/repos/$AUTHOR/$REPOSITORY/releases/latest" | grep '"tag_name":' | sed -E 's/.*"([^"]+)".*/\1/')
# Print version
echo "Download $TAG_VERSION last version $REPOSITORY"
# Download Adafruit library
if [ ! -d "$LIBRARY_PATH" ] ; then
    echo "Make folder $LIBRARY_PATH"
    mkdir "$LIBRARY_PATH"
fi
# Download latest version
wget --output-document "$LIBRARY_PATH/latest.tar.gz" "https://github.com/$AUTHOR/$REPOSITORY/archive/$TAG_VERSION.tar.gz"
# Unzip file
tar -xf "$LIBRARY_PATH/latest.tar.gz" -C "$LIBRARY_PATH"
# Remove file
rm "$LIBRARY_PATH/latest.tar.gz"
# Rename folder
mv "$LIBRARY_PATH/$REPOSITORY-$TAG_VERSION" "$LIBRARY_PATH/$REPOSITORY"
# Download complete
echo "${green}$REPOSITORY added in $LIBRARY_PATH ${reset}"
# EOF
