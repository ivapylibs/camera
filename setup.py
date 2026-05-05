"""!
  @defgroup Camera  Camera interfaces
"""


from setuptools import setup, find_packages

setup(
    name="camera",
    version="2.2.0",
    description="Classes implementing the runner interface for the commonly used cameras",
    url="https://github.com/ivapylibs/camera",
    author="IVALab",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "pyrealsense2==2.54.2.5684",
        "numpy",
        "matplotlib",
        "opencv-contrib-python>=4.5.5.62",
        "scikit-learn",
        "tqdm",
        "improcessor @ git+https://github.com/ivapylibs/improcessor.git",
        "depthai==2.30.0.0",
        "ivapy @ git+https://github.com/ivapylibs/ivapy.git",
        "PyYAML",
        "yacs"
    ],
    extras_require={
        "viz": ["mayavi"],
    },
)
