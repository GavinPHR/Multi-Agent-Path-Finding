import setuptools

# read the contents of your README file
with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
  name = 'cbs-mapf',         # How you named your package folder (MyLib)
  packages=setuptools.find_packages(),
  version = '0.5',      # Start with a small number and increase it with every change you make
  license='MIT',        # Chose a license from here: https://help.github.com/articles/licensing-a-repository
  description = 'Anonymous Multi-Agent Path Finding with Conflict-Based Search and Space-Time A*',   # Give a short description about your library
  author = 'Haoran Peng',                   # Type in your name
  author_email = 'gavinsweden@gmail.com',      # Type in your E-Mail
  url = 'https://github.com/GavinPHR/Multi-Agent-Path-Finding',   # Provide either the link to your github or to your website
  keywords = ['multi-agent-path-finding', 'anonymous-agent', 'conflict-based-search', 'mapf', 'cbs', 'a-star'],   # Keywords that define your package best
  install_requires=[            # I get to this in a second
          'numpy',
          'scipy',
          'space-time-astar'
      ],
  classifiers=[
    'Development Status :: 4 - Beta',      # Chose either "3 - Alpha", "4 - Beta" or "5 - Production/Stable" as the current state of your package
    'Intended Audience :: Developers',      # Define that your audience are developers
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',   # Again, pick a license
  ],
  python_requires='>=3.5',
  long_description=long_description,
  long_description_content_type='text/markdown'
)

# python3 setup.py sdist bdist_wheel
# twine upload dist/*
# Tutorial at https://medium.com/@joel.barmettler/how-to-upload-your-python-package-to-pypi-65edc5fe9c56