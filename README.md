# Camera

Encapsulation of typical camera functionality.

## Install

### Dependencies

These dependencies are handled via `setup.py` dependency resolution.
You may install them from source too by reading their corresponding README.

- [improcessor](https://github.com/ivapylibs/improcessor)

#### Visualization

If you use the `viz` extras install, you must have a suitable gui backend for `mayavi` to work.

```bash
ImportError: Could not import backend for traitsui.  Make sure you
        have a suitable UI toolkit like PyQt/PySide or wxPython
        installed.
```

Install either pyqt5 (or wxpython) to resolve this error.

```bash
# upgrade pip, or you may get stuck on license agreement
# https://stackoverflow.com/questions/66546886/pip-install-stuck-on-preparing-wheel-metadata-when-trying-to-install-pyqt5
pip install --upgrade pip
pip install PyQt5
```

If you have issues with PyQt, you can try wxpython instead.

```bash
# https://github.com/wxWidgets/Phoenix/issues/2225
pip install "wxpython<4.2.0"
```

If a suitable ui backend is not found or there are other issues with `mayavi`, visualizations will fall back to matplotlib.


### Install

```bash
git clone https://github.com/ivapylibs/camera.git
pip3 install -e camera[viz]

# or if inside of the checked out directory
pip3 install -e .[viz]
```

You may be interested in installing this in a virtual environment, which helps isolate packages from your system python.

```bash
python3 -m venv venv
source venv/bin/activate
pip3 install -e camera[viz]

# or if inside of the checked out directory
python3 -m venv venv
source venv/bin/activate
pip3 install -e .[viz]
```

## Note:

### `python -m camera.d435.testing.tabletop_plane`

   This testing file uses a 3d visualization package `mayavi`, which is for helping some undergrads understand the algorithm in a course.
   Please install the dependency first:

   ```bash
   pip install -e .[viz]
   ```

   The test file visualize the extracted tabletop plane point cloud in the camera frame, which is a necessary step for the height estimation.
   You can skip that testing file and run `python -m camera.d435.testing.height` directly.
