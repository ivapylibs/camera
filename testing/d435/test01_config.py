#!/usr/bin/python
#============================= test01_config =============================
'''!
@brief  Test out the configuration node class for the D435 camera.

There are two test01 files, one for the CfgD435 class and one for the
main D435 interface class.  This is the one for the configuration.
'''
#============================= test01_config =============================

#
# @author   Patricio A. Vela,       pvela@gatech.edu
# @date     2023/05/26
#
# NOTE: 4 space tabs with conversion, 2 space indent, 80 column.
#============================= test01_config =============================


from camera.d435.runner2 import CfgD435


defDict = CfgD435.get_default_settings()

print('\nSettings as a dictionary for use by configuration node.')
print(defDict)

conf1 = CfgD435(init_dict = defDict)
print('\nSettings as applied from dictionary for the configuration node.')
print(conf1)

print('\nAbove and below YAML setting specs should be the same.\n')
conf2 = CfgD435()
print(conf2)


#
#============================= test01_config =============================
