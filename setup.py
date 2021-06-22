from distutils.core import setup
setup(
    name="DepthVisualizer",
    packages=["DepthVisualizer"],
    version='0.24',
    license="MIT",
    description="Point Cloud & Depth Map visualization library",
    author="Eren Balatkan",
    author_email="e.balatkan@gmail.com",
    url="https://github.com/Navhkrin/DepthVisualizer",
    download_url="https://github.com/Navhkrin/DepthVisualizer/archive/v0.2.tar.gz",
    keywords=["Point Cloud", "Depth Map", "OpenGL", "Renderer"],
    install_requires=[
        'glfw',
        'PyOpenGL',
        'PyGLM',
        'numpy'
    ],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
    ],
)