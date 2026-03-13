import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ayadiab/chair_ws/src/medical_wheelchair/install/medical_wheelchair'
