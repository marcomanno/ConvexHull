#! python3

import os
import subprocess
import traceback
import sys
def win() : return sys.platform == "win32"
def linux() : return sys.platform == "linux"

def ensureDirectoryExists(path):
    "ensureDirectoryExists"
    try:
        os.makedirs(path)
    except OSError:
        if not os.path.isdir(path):
            raise

# config = Release or Debug
# sanitizer = Allowed GCC7 values for -fsanitize option. E.g. "undefined" or "address".
def run(config, sanitizer=None):
    SCRIPT_DIR = os.path.dirname(os.path.abspath(os.path.abspath(__file__)))
    SOURCE_DIR = os.path.join(SCRIPT_DIR, '..')
    
    CURR_DIR = os.curdir
    OUT_DIR = os.path.join(SCRIPT_DIR, "..", 'out')
    
    print(OUT_DIR)
    
    ensureDirectoryExists(OUT_DIR)
    
    os.chdir(OUT_DIR)
    
    cmake_cmd = "cmake"
    if linux():
        cmake_cmd += "3"
    

    CMAKE_COMMAND = ""
    if win():
        CMAKE_COMMAND = [cmake_cmd, "-G", "Visual Studio 16 2019", "-A",  "x64", SOURCE_DIR]
    elif linux():
        p = subprocess.Popen(["g++", "--version"], stdout=subprocess.PIPE)
        result = p.communicate()
        #print(result)
        if result[0].find(b"8.3") < 0:
            print("### ERROR: expected gcc version 8.3.1")
            print("### Enable gcc 8 using:   scl enable devtoolset-8 bash")
            sys.exit()
        CMAKE_COMMAND = [cmake_cmd, "-G", "Eclipse CDT4 - Unix Makefiles", "-DCMAKE_BUILD_TYPE=" + config]
        if sanitizer != None:
            CMAKE_COMMAND.append("-DSANITIZER=" + sanitizer)
        CMAKE_COMMAND.append(SOURCE_DIR)

    subprocess.check_call(CMAKE_COMMAND)

    os.chdir(CURR_DIR)

if __name__ == '__main__':
    try:
        # For Windows builds, we just need to generate the visual studio and
        # whether it is release or debug doesn't matter at this stage.
        # For Linux builds, it does matter.
        run("Release")
    except Exception:
        traceback.print_exc()


