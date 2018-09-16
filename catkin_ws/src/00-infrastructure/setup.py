from setuptools import setup, find_packages

packages = []
packages.extend(find_packages('duckietown/include'))
packages.extend(find_packages('easy_algo/include'))
packages.extend(find_packages('easy_logs/include'))
packages.extend(find_packages('easy_node/include'))
packages.extend(find_packages('easy_regression/include'))
packages.extend(find_packages('what_the_duck/include'))

setup(name='DuckietownUtils',
      url='http://github.com/duckietown/Software',
      maintainer="Andrea Censi",
      maintainer_email="andrea@duckietown.org",
      description='',
      long_description='',
      keywords="Optimization",
      classifiers=[
          'Development Status :: 4 - Beta',
      ],

      version='1.0',
      package_dir={
          'duckietown_utils': 'duckietown/include/duckietown_utils',
          'duckietown_utils_tests': 'duckietown/include/duckietown_utils_tests',
          'easy_algo': 'easy_algo/include/easy_algo',
          'easy_algo_tests': 'easy_algo/include/easy_algo_tests',
          'easy_logs': 'easy_logs/include/easy_logs',
          'easy_logs_tests': 'easy_logs/include/easy_logs_tests',

          'easy_node': 'easy_node/include/easy_node',
          'easy_node_tests': 'easy_node/include/easy_node_tests',
          'easy_regression': 'easy_regression/include/easy_regression',
          'easy_regression_tests': 'easy_regression/include/easy_regression_tests',
          'what_the_duck': 'what_the_duck/include/what_the_duck',
          'what_the_duck_tests': 'what_the_duck/include/what_the_duck_tests',
      },
      packages=packages,
      install_requires=[

      ],

      tests_require=[

      ],

      # This avoids creating the egg file, which is a zip file, which makes our data
      # inaccessible by dir_from_package_name()
      zip_safe=False,

      # without this, the stuff is included but not installed
      include_package_data=True,

      entry_points={

          'console_scripts': [
              # 'mcdp-plot = mcdp_cli:mcdp_plot_main',
              'dt-logs-download = easy_logs.cli:main_download',
              'dt-logs-copy = easy_logs.cli:main_copy',
              'dt-logs-details = easy_logs.cli:main_details',
              'dt-logs-gallery = easy_logs.cli:main_gallery',
              'dt-logs-ipfs-pack = easy_logs.cli:main_ipfs_pack',
              'dt-logs-summary = easy_logs.cli:main_summary',
              'dt-logs-find = easy_logs.cli:main_find',
              'dt-logs-thumbnails = easy_logs.cli:main_thumbnails',
              'dt-logs-videos = easy_logs.cli:main_videos',
          ]
      }
      )
