#!/usr/bin/env python3

import sys
from rqt_gui.main import Main
from temperature_control_gui.temperature_control_gui import TemperatureControlGUI

def main():
    main = Main()
    sys.exit(main.main(standalone='temperature_control_gui.temperature_control_gui:TemperatureControlGUI'))

if __name__ == '__main__':
    main()
