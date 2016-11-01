/*
 *    Copyright 2016 Rethink Robotics
 *
 *    Copyright 2016 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

'use strict';

let fs = require('fs');
let path = require('path');

let cmakePath = process.env.CMAKE_PREFIX_PATH;
let cmakePaths = cmakePath.split(':');
let jsMsgPath = 'share/gennodejs/ros';

let packagePaths = {};

module.exports = function (messagePackage) {
  if (packagePaths.hasOwnProperty(messagePackage)) {
    return packagePaths[messagePackage];
  }
  // else
  const found = cmakePaths.some((cmakePath) => {
    let path_ = path.join(cmakePath, jsMsgPath, messagePackage, '_index.js');
    if (fs.existsSync(path_)) {
      packagePaths[messagePackage] = require(path_);
      return true;
    }
    return false;
  });
  if (found) {
    return packagePaths[messagePackage];
  }
  // else
  throw new Error('Unable to find message package ' + messagePackage + ' from CMAKE_PREFIX_PATH');
};
