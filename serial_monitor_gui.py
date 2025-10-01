import pygame
import pygame_gui
from collections import deque
import random
import time

import serial
import serial.tools.list_ports
import os
import re

from pygame_gui import UIManager, PackageResource

from pygame_gui.elements import UIButton
from pygame_gui.elements import UITextEntryLine
from pygame_gui.elements import UIDropDownMenu
from pygame_gui.elements import UILabel
from pygame_gui.elements.ui_text_box import UITextBox

from pygame_gui.windows import UIMessageWindow

import gvar_ctrl

enable_serial_monitor = 1 # 0-disable 1-in app 2-in terminal
serial_log_file = None
serial_msg_text = ""
joysticks = None


class Options:
    def __init__(self):
        self.resolution = (600, 800)
        self.fullscreen = False

class OptionsUIApp:
    def __init__(self):
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (100,45)
        pygame.init()
        pygame.display.set_caption("Options UI")
        self.options = Options()

        if self.options.fullscreen:
            self.window_surface = pygame.display.set_mode(self.options.resolution,
                                                          pygame.FULLSCREEN)
        else:
            self.window_surface = pygame.display.set_mode(self.options.resolution)

        self.background_surface = None

        self.ui_manager = UIManager(self.options.resolution,
                                    PackageResource(package='data.themes',
                                                    resource='theme_2.json'))
        self.ui_manager.preload_fonts([{'name': 'fira_code', 'point_size': 10, 'style': 'bold'},
                                       {'name': 'fira_code', 'point_size': 10, 'style': 'regular'},
                                       {'name': 'fira_code', 'point_size': 10, 'style': 'italic'},
                                       {'name': 'fira_code', 'point_size': 14, 'style': 'italic'},
                                       {'name': 'fira_code', 'point_size': 14, 'style': 'bold'}
                                       ])

        
        self.test_drop_down = None

        self.serial_monitor_mode_header_textbox = None
        self.serial_monitor_mode = None

        self.serial_select_dropdown = None
        self.serial_connect_button = None
        self.serial_refresh_button = None
        self.serial_baudrate_textbox = None

        self.serial_msg_disp = None
        self.serial_msg_entry = None

        self.recreate_ui()

        self.clock = pygame.time.Clock()
        self.time_delta_stack = deque([])

        self.button_response_timer = pygame.time.Clock()
        self.running = True
        self.debug_mode = False

        self.all_enabled = True
        self.all_shown = True

    def recreate_ui(self):
        self.ui_manager.set_window_resolution(self.options.resolution)
        self.ui_manager.clear_and_reset()

        self.background_surface = pygame.Surface(self.options.resolution)
        self.background_surface.fill(self.ui_manager.get_theme().get_colour('dark_bg'))

        current_resolution_string = (str(self.options.resolution[0]) +
                                     'x' +
                                     str(self.options.resolution[1]))
        self.test_drop_down = UIDropDownMenu(options_list=['1024x768', '1200x800', '1440x800', '1600x900', '800x600', '600x800'],
                                             starting_option=current_resolution_string,
                                             relative_rect = pygame.Rect((int(self.options.resolution[0] * 0.01),
                                                          int(self.options.resolution[1] * 0.01)),
                                                         (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
                                             manager = self.ui_manager)

        
        self.serial_select_dropdown = UIDropDownMenu(
            options_list=['not selected'],
            starting_option='not selected',
            relative_rect = pygame.Rect((int(self.options.resolution[0] * 0.99) - int(self.options.resolution[0] / 4),
                            int(self.options.resolution[1] * 0.01)),
                            (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
            manager = self.ui_manager,
            )
        
        self.serial_refresh_button = UIButton(
            pygame.Rect((int(self.options.resolution[0] * 0.99) - int(self.options.resolution[0] / 4),
                            int(self.options.resolution[1] * 0.01)+int(self.options.resolution[1] / 32)),
                            (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
            'Refresh COM Ports',
            self.ui_manager)

        self.serial_connect_button = UIButton(
            pygame.Rect((int(self.options.resolution[0] * 0.99) - int(self.options.resolution[0] / 4),
                            int(self.options.resolution[1] * 0.01)+int(self.options.resolution[1] / 16)),
                            (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
            'Press to Connect',
            self.ui_manager)
        
        self.serial_baudrate_textbox = UITextEntryLine(
            relative_rect=pygame.Rect((int(self.options.resolution[0] * 0.99) - int(self.options.resolution[0] / 4),
                            int(self.options.resolution[1] * 0.01)+int(self.options.resolution[1] / 32 * 3)),
                            (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
            manager=self.ui_manager,
            initial_text="115200"
        )
        
        self.serial_monitor_mode_header_textbox = UILabel (
            relative_rect= pygame.Rect((int(self.options.resolution[0] * 0.01),
                        int(self.options.resolution[1] * 0.01)+int(self.options.resolution[1] / 32 * 1)),
                        (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
            text="Select serial output method"
        )

        self.serial_monitor_mode = UIDropDownMenu(
            options_list=['In app', 'In terminal', 'Disable'],
            starting_option='In app',
            relative_rect= pygame.Rect((int(self.options.resolution[0] * 0.01),
                        int(self.options.resolution[1] * 0.01)+int(self.options.resolution[1] / 32 * 2)),
                        (int(self.options.resolution[0] / 4), int(self.options.resolution[1] / 32))),
            manager=self.ui_manager,
            object_id="#serialmonitor"
        )

        self.serial_msg_disp = UITextBox(
            html_text="",
            relative_rect=pygame.Rect((int(self.options.resolution[0] * 0.01 + self.options.resolution[0] / 4),
                        int(self.options.resolution[1] * 0.01 + self.options.resolution[1] / 32)),
                        (int(self.options.resolution[0] / 2 - self.options.resolution[0] * 0.02), int(self.options.resolution[1] * 28 / 32))),
            manager=self.ui_manager
        )

        self.serial_msg_entry = UITextEntryLine(
            relative_rect=pygame.Rect((int(self.options.resolution[0] * 0.01 + self.options.resolution[0] / 4),
                        int(self.options.resolution[1] * 0.01)),
                        (int(self.options.resolution[0] / 2 - self.options.resolution[0] * 0.02), int(self.options.resolution[1] / 32))), 
            manager=self.ui_manager,
            object_id='#serial_text_entry'
        )

        self.serial_msg_entry.set_text('')

    def create_message_window(self):
        self.button_response_timer.tick()
        self.message_window = UIMessageWindow(
            rect=pygame.Rect((random.randint(0, self.options.resolution[0] - 300),
                              random.randint(0, self.options.resolution[1] - 200)),
                             (300, 250)),
            window_title='Test Message Window',
            html_message='this is a message',
            manager=self.ui_manager)
        time_taken = self.button_response_timer.tick() / 1000.0
        # currently taking about 0.35 seconds down from 0.55 to create
        # an elaborately themed message window.
        # still feels a little slow but it's better than it was.
        print("Time taken to create message window: " + str(time_taken))

    def check_resolution_changed(self):
        print(self.test_drop_down.selected_option)
        resolution_string = self.test_drop_down.selected_option[0].split('x')
        resolution_width = int(resolution_string[0])
        resolution_height = int(resolution_string[1])
        if (resolution_width != self.options.resolution[0] or
                resolution_height != self.options.resolution[1]):
            self.options.resolution = (resolution_width, resolution_height)
            self.window_surface = pygame.display.set_mode(self.options.resolution)
            self.recreate_ui()

    def process_events(self):
        global enable_serial_monitor
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            self.ui_manager.process_events(event)

            if (event.type == pygame_gui.UI_TEXT_ENTRY_FINISHED):
                if (event.ui_object_id == '#main_text_entry'):
                    print("main: " + event.text)
                if (event.ui_object_id == '#serial_text_entry'):
                    try:
                        str_to_be_sent = event.text
                        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
                        print("Sent: ", end="")
                        print(utf8str)
                        gvar_ctrl.mcu_serial_object.write(utf8str)
                        self.serial_msg_entry.set_text("")
                    except:
                        print("Send failed!")
                    

            if event.type == pygame_gui.UI_BUTTON_PRESSED:
                if event.ui_element == self.serial_connect_button:
                    try:
                        print("--Connecting to {}".format(self.serial_select_dropdown.selected_option[0]))
                        conn_baud = int(self.serial_baudrate_textbox.get_text())
                        gvar_ctrl.mcu_serial_object = serial.Serial(port=self.serial_select_dropdown.selected_option[0], baudrate=conn_baud, timeout=.1)
                    except:
                        print("--Connection failed")
                    else:
                        print("--Connected")

                if event.ui_element == self.serial_refresh_button:
                    try:
                        print(self.serial_select_dropdown.options_list)
                        self.serial_select_dropdown.options_list = [('not selected', 'not selected')]
                        ports = serial.tools.list_ports.comports()
                        for port, desc, hwid in sorted(ports):
                                self.serial_select_dropdown.add_options([str(port)])
                    except:
                        print("i don't know why but refresing failed")

            if (event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED):
                if (event.ui_element == self.test_drop_down):
                    self.check_resolution_changed()
                if (event.ui_element == self.serial_monitor_mode):
                    # ['In app', 'In terminal', 'Disable']
                    if self.serial_monitor_mode.selected_option == 'In app':
                        enable_serial_monitor = 1
                    elif self.serial_monitor_mode.selected_option == 'In terminal':
                        enable_serial_monitor = 2
                    elif self.serial_monitor_mode.selected_option == 'Disable':
                        enable_serial_monitor = 0
                    else:
                        enable_serial_monitor = 0

    def run(self):
        global joysticks
        global enable_serial_monitor
        global serial_log_file
        global serial_msg_text

        global context

        serial_msg_text_size = 200

        while self.running:
            time_delta = self.clock.tick() / 1000.0
            self.time_delta_stack.append(time_delta)
            if len(self.time_delta_stack) > 2000:
                self.time_delta_stack.popleft()
            # check for input
            self.process_events()
            # respond to input
            self.ui_manager.update(time_delta)
            # draw graphics
            self.window_surface.blit(self.background_surface, (0, 0))
            self.ui_manager.draw_ui(self.window_surface)
            pygame.display.update()

            # pyserial
            if (enable_serial_monitor != 0) and (gvar_ctrl.mcu_serial_object is not None):
                numBytesToRead = gvar_ctrl.mcu_serial_object.in_waiting
                if(numBytesToRead > 0):
                    line_readed = gvar_ctrl.mcu_serial_object.readline()
                    ms_rightnow = int(round(time.time() * 1000))
                    serial_log_file.write(str(ms_rightnow) + ", " + str(line_readed)+'\n')
                    if enable_serial_monitor == 1: # in app option
                        serial_msg_text += (str(line_readed)+'\n')
                        self.serial_msg_disp.appended_text += (str(line_readed)+'\n')
                        self.serial_msg_disp.set_text(serial_msg_text)
                        if self.serial_msg_disp.get_text_letter_count() > serial_msg_text_size:
                            diff = self.serial_msg_disp.get_text_letter_count()
                            serial_msg_text = serial_msg_text[diff:]
                        
                        if self.serial_msg_disp.scroll_bar is not None:
                            # set the scroll bar to the bottom
                            percentage_visible = (self.serial_msg_disp.text_wrap_rect[3] /
                                                self.serial_msg_disp.text_box_layout.layout_rect.height)
                            self.serial_msg_disp.scroll_bar.start_percentage = 1.0 - percentage_visible
                            self.serial_msg_disp.scroll_bar.scroll_position = (self.serial_msg_disp.scroll_bar.start_percentage *
                                                            self.serial_msg_disp.scroll_bar.scrollable_height)
                            self.serial_msg_disp.redraw_from_text_block()
                    if enable_serial_monitor == 2: # in terminal option
                        print(line_readed)
                    
def open_serial_log(log_path="./serial_log"):
    global serial_log_file
    try:
        dir_list = os.listdir(log_path)
    except Exception as e:
        print("open_serial_log: ", e)
        print("Creating a folder with name " + log_path)
        os.mkdir(log_path)
        dir_list = os.listdir(log_path)
    dir_list = [f for f in dir_list if os.path.isfile(log_path+'/'+f)]

    largest_index = 0

    for each_log in dir_list:
        # getting numbers from string 
        temp = re.findall(r'\d+', each_log)
        res = list(map(int, temp))
        largest_index = max([largest_index, res[0]])

    
    serial_log_file = open(log_path + '/serial_log_' + str(largest_index+1) + '.txt', 'w')
        


if __name__ == '__main__':
    ports = serial.tools.list_ports.comports()
    open_serial_log()
    for port, desc, hwid in sorted(ports):
            print("{}: [{}]".format(port, hwid))

    app = OptionsUIApp()
    app.run()
