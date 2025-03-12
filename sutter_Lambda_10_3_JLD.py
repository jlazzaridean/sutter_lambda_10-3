import serial

class Controller:
    '''
    Basic device adaptor for Sutter Lambda 10-3 optical filter changer and
    SmartShutter® control system. Many more commands are available and have
    not been implemented.

    This adapter supports up to two SmartShutters and two 25 mm filter
    wheels connected to the Lambda 10-3 controller (positions 'A' and 'B'
    for each). It has been tested with 2 standalone filter wheels and 1
    standalone SmartShutter.

    This adapter was tested with USB communication. Your controller has
    USB communication enabled if there is a "U" in the upper right hand
    corner of the screen when you power it on. To enable USB
    communication, enter the following sequence on the keypad:
        LOCAL -> MODE -> 7 1 4 1 0 (& then power cycle).
    '''
    def __init__(self, which_port, name='Lambda 10-3', verbose=False):
        self.name = name
        self.verbose = verbose
        if self.verbose:
            print('%s: Opening filter wheel and shutter controller'%self.name)
        try:
            self.port = serial.Serial(port=which_port, baudrate=128000,
                                      timeout=5)
        except serial.serialutil.SerialException:
            raise IOError('%s: unable to connect on %s'%(
                self.name, which_port))
        self.port.write(b'\xFD') # get controller type and configuration
        configuration = self.port.readline()
        assert len(configuration) != 36, (
            '%s: configurations with shutter C are not supported', self.name)
        assert len(configuration) == 31
        config_s = configuration[5:].decode('ASCII')
        # Parse the configuration to determine what is connected
        wa = config_s[0:5]; wb = config_s[5:10]; wc = config_s[10:15]
        sa = config_s[15:20]; sb = config_s[20:25]
        assert wc == "WC-NC", (
            '%s: configurations with wheel C are not supported', self.name)
        assert wa[:3]=='WA-' and wb[:3]=='WB-'
        assert sa[:3]=='SA-' and sb[:3]=='SB-'
        for w in [wa, wb]:
            assert w[3:] == '25' or w[3:] == 'NC', (
                '%s: Only 25 mm filter wheels are supported', self.name)
        for s in [sa, sb]:
            assert s[3:] == 'IQ' or s[3:] == 'VS', (
                '%s: Unrecognized shutter configuration', self.name)
        self.wheels=[]
        self.shutters=[]
        if wa == 'WA-25': self.wheels.extend([0]) # wheel A
        if wb == 'WB-25': self.wheels.extend([1]) # wheel B
        if sa == 'SA-IQ': self.shutters.extend([0]) # shutter A
        if sb == 'SB-IQ': self.shutters.extend([1]) # shutter B
        self.names_to_channels = {'A': 0, 'B': 1}
        self.channels_to_names = {0: 'A', 1: 'B'}
        wheel_names = [self.c2n(x) for x in self.wheels]
        shutter_names = [self.c2n(x) for x in self.shutters]
        if self.verbose:
            if len(self.wheels) > 0 and len(self.shutters) > 0:
                print('%s: Filter wheels connected:'%self.name, wheel_names)
                print('%s: Shutters connected:'%self.name, shutter_names)
            elif len(self.wheels) == 0 and len(self.shutters) > 0:
                print('%s: No filter wheels connected'%self.name)
                print('%s: Shutters connected:'%self.name, shutter_names)
            elif len(self.wheels) > 0 and len(self.shutters) == 0:
                print('%s: Filter wheels connected:'%self.name, wheel_names)
                print('%s: No shutters connected'%self.name)
            else:
                raise RuntimeError(
                    '%s: No filter wheels or shutters connected'%self.name)
        self._pending_cmd = None
        self.wheel_target_positions = [0]*len(self.wheels)
        for wheel in self.wheels:
            self.move(0, wheel=wheel) # use as shutter
        for shutter in self.shutters:
            self.set_shutter_mode('soft', shutter=shutter)
            self.change_shutter('closed', shutter=shutter)
        if self.verbose: print('%s: Done opening'%self.name)
        return None

    def move(self,
             position,      # maximum 10
             wheel=0,       # 0 (or 'A') or 1 (or 'B')
             speed=6,       # 6 is reliable
             block=True):   # True/False: wait for completion before returning
        wheel = self.n2c(wheel)
        assert wheel in self.wheels
        if self._pending_cmd is not None:
            self._finish_moving()
        assert position in range(10)
        assert speed in range(8)
        if self.verbose:
            print('%s: moving wheel %s to position %i (speed=%i)'%(
                self.name, self.c2n(wheel), position, speed))
        cmd = bytes([(wheel << 7) + (speed << 4) + position])
        self.port.write(cmd)
        self._pending_cmd = cmd
        self.wheel_target_positions[wheel] = position
        if block:
            self._finish_moving()
        return None

    def change_shutter(self,
                       state,       # 0 (or 'closed') or 1 (or 'open')
                       shutter=0,   # 0 (or 'A') or 1 (or 'B')
                       block=True):
        shutter = self.n2c(shutter)
        assert shutter in self.shutters
        assert state in ['closed', 'open', 0, 1]
        if self.verbose:
            print('%s: setting shutter %s to state %s'%(
                self.name, self.c2n(shutter), state))
        if self._pending_cmd is not None:
            self._finish_moving()
        if state == 'open' or state == 1:
            cmd = bytes([128 + 32 + (shutter << 4) + 8 + 2])
        elif state == 'closed' or state == 0:
            cmd = bytes([128 + 32 + (shutter << 4) + 8 + 4])
        self.port.write(cmd)
        self._pending_cmd = cmd
        if block:
            self._finish_moving()
        return None

    def set_shutter_mode(self,
                         mode,          # 'fast' or 'soft'
                         shutter=0,     # 0 (or 'A') or 1 (or 'B')
                         block=True):    
        shutter = self.n2c(shutter)
        assert shutter in self.shutters
        assert mode in ['fast', 'soft']
        if self.verbose:
            print('%s: setting shutter %s to mode %s'%(
                self.name, self.c2n(shutter), mode))
        if self._pending_cmd is not None:
            self._finish_moving()
        if mode == 'fast':
            cmd = bytes([220, (shutter+1)])
        elif mode == 'soft':
            cmd = bytes([221, (shutter+1)])
        self.port.write(cmd)
        self._pending_cmd = cmd
        if block:
            self._finish_moving()        
        return None

    def _finish_moving(self):
        if self._pending_cmd is None:
            return
        response = self.port.read(len(self._pending_cmd)+1)
        if response != self._pending_cmd + b'\r':
            print('%s: expected pending command =', self._pending_cmd)
            print('%s: response =', response)
            raise IOError('%s: unexpected response'%self.name)
        self._pending_cmd = None
        assert self.port.in_waiting == 0
        if self.verbose: print('%s: -> finished moving.'%self.name)
        return None

    def n2c(self, name_or_channel):
        if name_or_channel in self.names_to_channels.values():
            return name_or_channel
        elif name_or_channel in self.names_to_channels:
            return self.names_to_channels[name_or_channel]
        else:
            raise ValueError('Invalid filter wheel or shutter ID: %s'
                             % name_or_channel)

    def c2n(self, name_or_channel):
        if name_or_channel in self.channels_to_names.values():
            return name_or_channel
        elif name_or_channel in self.channels_to_names:
            return self.channels_to_names[name_or_channel]
        else:
            raise ValueError('Invalid filter wheel or shutter ID: %s'
                             % name_or_channel)  

    def close(self):
        for wheel in self.wheels:
            self.move(0, wheel=wheel) # use as shutter
        for shutter in self.shutters:
            self.change_shutter('closed', shutter=shutter)
        self.port.close()
        if self.verbose: print('%s: Closed.'%self.name)
        return None

if __name__ == '__main__':
    import time
    import random

    controller = Controller(which_port='COM13', verbose=True)

    # cycle the shutter in fast (high pitched!) mode
    for sh in controller.shutters:
        controller.set_shutter_mode(mode='fast', shutter=sh)
        print('\n# Cycling shutter in fast mode, first mostly open...')
        for i in range(3):
            controller.change_shutter(state='open', shutter=sh)
            time.sleep(1)
            controller.change_shutter(state='closed', shutter=sh)
        print('\n# ...and now mostly closed.')
        for i in range(3):
            controller.change_shutter(state='closed', shutter=sh)
            time.sleep(1)
            controller.change_shutter(state='open', shutter=sh)

    # cycle the shutter in soft (quiet!) mode
    for sh in controller.shutters:
        controller.set_shutter_mode(mode='soft', shutter=sh)
        print('\n# Cycling shutter in soft mode, first mostly open...')
        for i in range(3):
            controller.change_shutter(state='open', shutter=sh)
            time.sleep(1)
            controller.change_shutter(state='closed', shutter=sh)
        print('\n# ...and now mostly closed.')
        for i in range(3):
            controller.change_shutter(state='closed', shutter=sh)
            time.sleep(1)
            controller.change_shutter(state='open', shutter=sh)

    # performance tests for wheels:
    if len(controller.wheels) > 0:
        print('\n# Adjacent (fastest) move:')
        for wheel in controller.wheels:
            t0 = time.perf_counter()
            controller.move(1, wheel=wheel)
            print('(time: %0.4fs)'%(time.perf_counter() - t0))

        print('\n# Opposite (slowest) move:')
        for wheel in controller.wheels:
            t0 = time.perf_counter()
            controller.move(6, wheel=wheel)
            print('(time: %0.4fs)'%(time.perf_counter() - t0))    

        print('\n# Non blocking call:')
        for wheel in controller.wheels:
            t0 = time.perf_counter()
            controller.move(0, wheel=wheel, block=False)
            print('(time: %0.4fs)'%(time.perf_counter() - t0))
            print('(do something else...)')
            print('# Finish call:')
            controller._finish_moving()
            print('(time: %0.4fs, incl. prints)\n'%(time.perf_counter() - t0))

    if len(controller.wheels) > 1:
        print('\n# Non blocking call with 2 wheels:')
        for i in range(10):
            t0 = time.perf_counter()
            controller.move(6, wheel=0, block=False) # move to 6 is slower...
            controller.move(1, wheel=1, block=False) # calls _finish_moving()
            print('(time: %0.4fs)'%(time.perf_counter() - t0))
            print('(do something else...)')
            print('# Finish call:')
            controller._finish_moving()
            print('(time: %0.4fs, incl. prints)\n'%(time.perf_counter() - t0))
            for wheel in controller.wheels:
                controller.move(0, wheel=wheel) # reset

##    # reliability tests:
##    moves = 10
##    for wheel in controller.wheels:
##        controller.verbose = True
##        for i in range(moves):
##            position = i%10
##            controller.move(position, wheel=wheel)
##        for i in range(moves):
##            position = random.randint(0, 9)
##            controller.move(position, wheel=wheel)
##
##        controller.verbose = False
##        for i in range(moves):
##            position = i%10
##            controller.move(position, wheel=wheel)
##        for i in range(moves):
##            position = random.randint(0, 9)
##            controller.move(position, wheel=wheel)

    controller.close()
