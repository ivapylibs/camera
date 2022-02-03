# Camera
Encaspulation of typical camera functionality.

## Install
```
git clone https://github.com/ivapylibs/camera.git
pip3 install -e camera/
```



## Ubuntu Python Environment Setup

The suggested python version is python 3.7, because one of the package (```pptk```) used in the test script is only supported by the python 3.7. The remaining will use the python 3.7 as an example. Any other version is similar.

1. Install the Python 3.7

   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository ppa:deadsnakes/ppa
   sudo apt update
   sudo apt install python3.7
   ```

   Verify the installation:

   ```bash
   python3.7
   ```

2. Install the Pip 3.7. The command line installation often cause errors, so use the installation script

   ```bash
   cd
   curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
   python3.7 get-pip.py
   # should enter the python3.7 environment
   # Ctrl-D to exit
   ```

   Verify the installation:

   ```bash
   pip3.7 -V
   # should see something like below
   # pip xx.x.x from /default/site-packages/path
   ```

3. Change the default python3 version pip3  version (optional)

   Up until now every thing you run should be in the form of below **(include the commands in the install section above)**. Because if your system have multiple python versions, the ```python3``` or ```pip3``` command might not be linked to the one you want.

   ```bash
   pip3.7 install SOMEPACKAGE
   python3.7 SOMESCRIPT.py
   ```

   We can optionally create the symlink ```pip3``` and ```python3``` for our desired version so that you can use the following command instead:

   ```bash
   pip3 install SOMEPACKAGE
   python3 SOMESCRIPT.py
   ```

   One way to do it is to use the ```update-alternatives``` command (change the python version in the first two lines accordingly, and similar for the pip) :

   ```bash
   sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.5 1
   sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 2
   
   sudo update-alternatives --config python
   
   sudo update-alternatives  --set python /usr/bin/python3.7
   ```

   Alternatively, we can create an virtual environment using the [venv](https://docs.python.org/3/library/venv.html) or the [Anaconda](https://www.anaconda.com/) for the desired python version to avoid the this step.

