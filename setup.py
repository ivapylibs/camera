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
        "numpy",
        "matplotlib",
        "scikit-learn",
        "tqdm",
        "PyYAML",
        "yacs",
        "pyrealsense2>=2.54.2.5684",
        "opencv-contrib-python>=4.5.5.62",
        "depthai==2.30.0.0",
    ],
    extras_require={
        "viz": ["mayavi"],
    },
)
