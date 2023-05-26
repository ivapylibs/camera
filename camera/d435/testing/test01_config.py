#!/usr/bin/python
#============================= test01_config =============================
'''!
@brief  Test out the configuration node class for the D435 camera.

'''
#============================= test01_config =============================

#
# @author   Patricio A. Vela,       pvela@gatech.edu
# @date     2023/05/26
#
# NOTE: 4 space tabs with conversion, 2 space indent, 80 column.
#============================= test01_config =============================


from camera.d435.runner2 import CfgD435


x = dict( test=5, recurse = dict( x = 5, y = 6))
print(x)

defDict = CfgD435.get_default_settings()
print(defDict)

test1 = CfgD435(init_dict = defDict)
print(test1)

print('\nAbove and below YAML setting specs should be the same.\n')

test2 = CfgD435()
print(test2)


#
#============================= test01_config =============================
