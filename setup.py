from setuptools import setup, find_packages

setup(
    name="camera",
    version="2.1.0",
    description="Classes implementing the runner interface for the commonly used cameras",
    url="https://github.com/ivapylibs/camera",
    author="IVALab",
    packages=find_packages(),
    install_requires=[
        "pyrealsense2",
        "numpy",
        "matplotlib",
        "opencv-contrib-python==4.5.5.62",
        "scikit-learn",
        "tqdm",
        "improcessor @ git+https://github.com/ivapylibs/improcessor.git",
    ],
    extras_require={
      "viz": ["mayavi", "wxpython"]
    }
)
