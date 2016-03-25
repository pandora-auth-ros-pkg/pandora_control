# Software License Agreement
__version__ = "0.0.1"
__status__ = "Development"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__authors__ = "Zisis Konstantinos"
__maintainer__ = "Zisis Konstantinos"
__email__ = "zisikons@gmail.gr"


import os
import sys
import pickle

## -- Set and hold RlData directory path -- ##
__path__ = os.path.dirname(__file__)
## ---------------------------------------- ##


## ------------------- Usefull Path Definitions -------------------------- ##
__rl_data_dir__ = os.path.expanduser("~/.rl_data")

__pandora_ws_dir__ = os.path.expanduser("~/pandora_ws")

__pandora_control_repo_dir__ = os.path.expanduser(
    __pandora_ws_dir__ + "/src/pandora_control")

__kinodynamic_control_dir__ = __pandora_control_repo_dir__ + \
    "/pandora_motion_control/src/pandora_kinodynamic_control"

__rl_data_repo_dir__ = os.path.expanduser(__kinodynamic_control_dir__ +
    "/tables")
## ---------------------------------------------------------------------- ##



def exec_system_cmd(cmd):
  try:
    os.system(cmd)
  except:
    e = sys.exc_info()[0]
    print e
    return False
  finally:
    return True


def create_rlData_dir():
    if os.path.exists(__rl_data_dir__):
        pass
    else:
        print "\033[1;32mCreating dir [%s]\033[0m" % __rl_data_dir__
        cmd = "mkdir %s" % __rl_data_dir__
        exec_system_cmd(cmd)


##
#  Create symbolic link to repository
#
def link_rlData_repo():
  if os.path.exists(__rl_data_dir__):
    # Symlinc || directory allready exists
    print "\033[1;33mDir [%s] already exists\033[0m" % __rl_data_dir__
    pass
  else:
    cmd = "ln -s %s %s" % (__rl_data_repo_dir__, __rl_data_dir__)
    print "\033[1;32mCreating Symlink from %s to %s" \
        % (__rl_data_dir__, __rl_data_repo_dir__)
    exec_system_cmd(cmd)


##
#  Copies from local data dir to repo dir
##
def add_to_repo():
    if os.path.exists(__rl_data_repo_dir__):
        cmd = "cp  %s/* %s/" %(__rl_data_dir__, __rl_data_repo_dir__)
        exec_system_cmd(cmd)
        return True
    return False

def push_data_origin():
  branch = 'origin/master'
  commit_msg = 'Updated RL data'
  cmd = "cd %s/.. && git add -A . && git commit -m %s" + \
      "&& git push %" % (__rl_data_repo_dir__, commit_msg, branch)
  exec_system_cmd(cmd)

def loadData(filename):
    filepath = __rl_data_dir__+"/"+filename
    if os.path.exists(filepath) and os.path.isfile(filepath):
        fileObject = open(filepath, 'r')
        _av_table = pickle.load(fileObject)
        fileObject.close()
        return [_av_table,True]
    else:
        return [0,False]

def storeData(data,filename):
    create_rlData_dir()
    filepath = __rl_data_dir__+"/"+filename
    fileObject = open(filepath ,'w')
    pickle.dump(data,fileObject)
    fileObject.close()
