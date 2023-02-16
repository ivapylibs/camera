from setuptools import setup

setup(
    name="camera",
    version="2.0.0",
    description="Classes implementing the runner interface for the commonly used cameras",
    url="https://github.com/ivapylibs/camera",
    author="IVALab",
    packages=["camera"],
    install_requires=[
        "pyrealsense2",
        "numpy",
        "matplotlib",
        "opencv-contrib-python==4.5.5.62",
        "scikit-learn",
        "tqdm",
    ],
    extras_require={
      "viz": ["mayavi", "wxpython"]
    }
)
