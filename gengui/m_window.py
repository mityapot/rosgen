#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
from PyQt5.QtWidgets import (QWidget, QMainWindow, QLabel, QLineEdit,
    QGridLayout, QApplication, QPushButton,  QAction, qApp, QFileDialog, QMessageBox, QMenu)
from PyQt5.QtCore import pyqtSignal, QRegExp
from PyQt5.QtGui import QIcon, QRegExpValidator
import xml.etree.cElementTree as ET
from genkernel import package
from gengui.sub_pub_wizard import ManagerGui
import shutil
import json


class GenGui(QMainWindow):
    """
    GenGui class, main window of application
    """

    manager = None

    def __init__(self):
        """
        GenGui object constructor
        """

        super().__init__()
        self.initUI()

    def initUI(self):
        """
        Function initialisation of main window interface
        """

        self.wid = RosGenWidget()
        self.setCentralWidget(self.wid)
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&Файл')
        editMenu = menubar.addMenu('&Редактирование')
        self.create_menu_par('Менеджер подписчиков и издателей', self.wid.show_manager, fileMenu, 'Ctrl+M')
        self.create_menu_par('Очистить', self.wid.clear_all_lines, editMenu, 'Ctrl+D')
        self.create_menu_par('Загрузить данные из...', self.wid.open_fileDialog, fileMenu, 'Ctrl+F')
        self.create_menu_par('Сохранить как...', self.wid.save_fileDialog, fileMenu, 'Ctrl+S')
        self.create_menu_par('Выход', self.exit_app, fileMenu, 'Esc')
        self.statusbar = self.statusBar()
        self.statusbar.showMessage('Ожидание данных')
        self.wid.msg2Statusbar[str].connect(self.statusbar.showMessage)
        self.setGeometry(600, 200, 700, 400)
        self.setWindowTitle('Генератор шаблонов ROS-приложения')
        self.show()

    def create_menu_par(self, name, trig_func, menu, shrt_cut):
        """
        Function create actions in menu
        :param name: name of action
        :type name: str
        :param trig_func: name of action function
        :type trig_func: function
        :param  menu: menu object, menu in menubar
        :type menu: QMenu
        :param shrt_cut: shortcut for action, optional
        :type shrt_cut: str
        """

        createdAction = QAction(name, self)
        createdAction.setShortcut(shrt_cut)
        createdAction.triggered.connect(trig_func)
        menu.addAction(createdAction)
        return createdAction

    def exit_app(self):
        """
        Function call when application quit
        """

        if self.wid.changed is True:
            but = QMessageBox().question(self, 'Message', "Вы точно хотите выйти и не сохранить?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if but == QMessageBox.Yes:
                qApp.quit()
        else:
            qApp.quit()

    def closeEvent(self, QCloseEvent):
        self.exit_app()


class RosGenWidget(QWidget):
    """
    RosGenWidget class, main working widget
    """

    msg2Statusbar = pyqtSignal(str)
    full_ed_lines = list()
    pkg = None
    changed = False

    def __init__(self):
        """
        RosGenWidget object constructor
        """

        super().__init__()
        self.manager = ManagerGui()
        self.initUI()

    def initUI(self):
        """
        Function initialisation of widget interface
        """

        lbl_names = ['Название проекта', 'Версия', 'Директория', 'Описание', 'Автор', 'Почта', 'Дополнительные зависимости', 'Название ноды']# , 'Зависимости ноды']
        param_list = ['motor_driver', '0.0.0', '/home/mitya/catkin_ws/src/', 'The motor_driver package', 'A. Kozov',
                      'alexey@todo.todo', 'nav_msgs, geometry_msgs, tf, ', 'motor_driver_node',]
                      # 'nav_msgs/Odometry, geometry_msgs/Twist, tf/transform_broadcaster, ']
        labels = []
        for name in lbl_names:
            labels.append(QLabel(name))
        for i, ph in zip(range(len(labels)),  param_list):
            ed_line = QLineEdit()
            if i == 1:
                ed_line.setValidator(QRegExpValidator(QRegExp("^([0-9\.])*[0-9]$")))
            elif i == 5:
                ed_line.setValidator(QRegExpValidator(QRegExp("^([a-z0-9_-]+\.)*[a-z0-9_-]+@[a-z0-9_-]+(\.[a-z0-9_-]+)*\.[a-z]{2,6}$")))
            ed_line.setPlaceholderText(ph)
            if i != 0:
                ed_line.textEdited.connect(self.change_data)
            else:
                ed_line.textEdited.connect(self.change_pkg_name)
            self.full_ed_lines.append(ed_line)
        grid = QGridLayout()
        grid.setSpacing(5)
        for i in range(1, len(labels) + 1):
            for j in range(0, 2):
                if j == 0:
                 grid.addWidget(labels[i - 1], i, j)
                else:
                    grid.addWidget(self.full_ed_lines[i - 1], i, j)
        ch_dirButton = QPushButton(self)
        ch_dirButton.setIcon(QIcon('./icons/open_folder.png'))
        ch_dirButton.clicked.connect(self.ch_dirDialog)
        grid.addWidget(ch_dirButton, 3, 3)
        genButton = QPushButton("Сгенерировать")
        genButton.clicked.connect(self.generate)
        grid.addWidget(genButton, len(labels) + 2, 1)
        self.setLayout(grid)
        self.setMinimumSize(700, 400)
        self.show()

    def data_from_xml(self, xml_file):
        """
        Function import data from xml file
        :param xml_file: path to source xml
        :type xml_file: str
        """

        param_list = []
        dep_str = ''
        dep_node_str = ''
        sub_list = list()
        pub_list = list()
        tree = ET.ElementTree(file=xml_file)
        root = tree.getroot()
        for child in root:
            if child.tag == 'name':
                param_list.insert(0, child.text)
            if child.tag == 'version':
                param_list.insert(1, child.text)
            if child.tag == 'description':
                param_list.insert(3, child.text)
            if child.tag == 'maintainer':
                param_list.insert(4, child.text)
                param_list.insert(5, child.attrib['email'])
            if child.tag == 'dir':
                param_list.insert(2, child.text)
            if child.tag == 'depend':
                dep_str = dep_str + child.text + ', '
            if child.tag == 'node':
                node = child
        param_list.insert(6, dep_str)
        for child in node:
            if child.tag == 'name':
                param_list.insert(7, child.text)
            # if child.tag == 'depend':
            #     dep_node_str = dep_node_str + child.text + '/' + child.attrib['type'] + ', '
            if child.tag == 'subscribers':
                for sub in child:
                    sub_dict = dict()
                    for param in sub:
                        sub_dict[param.tag] = param.text
                    sub_list.append(sub_dict)
            if child.tag == 'publishers':
                for pub in child:
                    pub_dict = dict()
                    for param in pub:
                        pub_dict[param.tag] = param.text
                    pub_list.append(pub_dict)
        # param_list.insert(8, dep_node_str)
        print(param_list)
        print(sub_list)
        print(pub_list)
        for line, val in zip(self.full_ed_lines, param_list):
            line.setText(val)
        self.manager.wid.pub_list = pub_list
        self.manager.wid.sub_list = sub_list
        self.manager.wid.reload_table()
        self.changed = False
        self.msg2Statusbar.emit('Выполнена выгрузка данных из XML')

    def data_from_json(self, json_file):
        """
        Function import data from json file
        :param json_file: path to source json
        :type json_file: str
        """

        param_list = ['name', 'version', 'dir', 'description']
        push_param = list()
        dep_str = ''
        dep_node_str = ''
        with open(json_file) as f:
            param_dict = json.load(f)
        for i in range(len(param_list)):
            push_param.append(param_dict[param_list[i]])
        push_param.append(param_dict['maintainer']['name'])
        push_param.append(param_dict['maintainer']['email'])
        for dep in param_dict['depend']:
            dep_str = dep_str + dep + ', '
        push_param.append(dep_str)
        push_param.append(param_dict['node']['name'])
        # for dep_node in param_dict['node']['depend']:
        #     dep_node_str = dep_node_str + dep_node['name'] + '/' + dep_node['type'] + ', '
        # push_param.append(dep_node_str)
        for line, val in zip(self.full_ed_lines, push_param):
            line.setText(val)
        self.manager.wid.pub_list = param_dict['node']['publishers']
        self.manager.wid.sub_list = param_dict['node']['subscribers']
        self.manager.wid.reload_table()
        self.changed = False
        self.msg2Statusbar.emit('Выполнена выгрузка данных из JSON')

    def create_gen_xml(self, out_file):
        """
        Function export data from xml file
        :param out_file: path to output xml file
        :type out_file: str
        """

        param_list = []
        msg = []
        msg_type = []
        dep_node = []
        for line in self.full_ed_lines:
            param_list.append(line.text())
        dep_pkg = param_list[6].split(', ')
        if dep_pkg[len(dep_pkg) - 1] == '':
            dep_pkg.pop()
        # dep_node = param_list[8].split(', ')
        # dep_node.pop()
        for dep in self.manager.wid.sub_list:
            dep_node.append(dep['msg_type'])
        for dep in self.manager.wid.pub_list:
            dep_node.append(dep['msg_type'])
        for dep in dep_node:
            a, b = dep.split('::')
            msg.append(a)
            msg_type.append(b)
        print(param_list)
        f = open('../genkernel/templates/package_rosgen.xml')
        o = open(out_file, 'a')
        flag = 0
        while 1:
            line = f.readline()
            if not line: break
            for i in range(6):
                line = line.replace('[{0}]'.format(i), param_list[i])
            line = line.replace('[7]', param_list[7])
            if line.find('[6]') != -1:
                for dep in dep_pkg:
                    line_dep = '\t<depend>{0}</depend>\n'.format(dep)
                    o.write(line_dep)
                flag = 1
            elif line.find('[8]') != -1:
                for dep, tp in zip(msg, msg_type):
                    line_dep = '\t\t<depend type="{1}">{0}</depend>\n'.format(dep, tp)
                    o.write(line_dep)
                flag = 1
            elif line.find('<subscribers>') != -1:
                o.write('\t\t<subscribers>\n')
                for sub in self.manager.wid.sub_list:
                    o.write('\t\t\t<sub>\n')
                    o.write('\t\t\t\t<name>{0}</name>\n'.format(sub['name']))
                    o.write('\t\t\t\t<msg_type>{0}</msg_type>\n'.format(sub['msg_type']))
                    o.write('\t\t\t\t<topic_name>{0}</topic_name>\n'.format(sub['topic_name']))
                    o.write('\t\t\t\t<queue_size>{0}</queue_size>\n'.format(sub['queue_size']))
                    o.write('\t\t\t</sub>\n')
                o.write('\t\t</subscribers>\n')
                flag = 1
            elif line.find('<publishers>') != -1:
                o.write('\t\t<publishers>\n')
                for pub in self.manager.wid.pub_list:
                    o.write('\t\t\t<pub>\n')
                    o.write('\t\t\t\t<name>{0}</name>\n'.format(pub['name']))
                    o.write('\t\t\t\t<msg_type>{0}</msg_type>\n'.format(pub['msg_type']))
                    o.write('\t\t\t\t<topic_name>{0}</topic_name>\n'.format(pub['topic_name']))
                    o.write('\t\t\t\t<queue_size>{0}</queue_size>\n'.format(pub['queue_size']))
                    o.write('\t\t\t</pub>\n')
                o.write('\t\t</publishers>\n')
                flag = 1
            if flag == 0:
                o.write(line)
            else:
                flag = 0
        o.close()
        f.close()
        self.changed = False

    def create_gen_json(self, out_file):
        """
        Function export data from json file
        :param out_file: path to output json file
        :type out_file: str
        """

        params = self.create_package_dict()
        with open(out_file, 'w') as fp:
            json.dump(params, fp)

    def check_data(self):
        """
        Function check data in line edit input
        :return: correct data or not
        :rtype: bool
        """

        for i in range(len(self.full_ed_lines)):
            if self.full_ed_lines[i].text() != "":
                if self.full_ed_lines[i].hasAcceptableInput():
                    continue
                else:
                    if i == 1:
                        self.msg2Statusbar.emit('Неправильный формат версии! Исправьте и повторите действие!')
                    elif i == 5:
                        self.msg2Statusbar.emit('Неправильная почта! Исправьте и повторите действие!')
                    return False
            else:
                self.msg2Statusbar.emit('Не все поля заполнены! Исправьте и повторите действие!')
                return False
        return True

    def create_package_dict(self):
        """
        Function create package parameters from line edit input
        :return: package parameters
        :rtype: dict
        """
        dep_node = list()
        param_list = ['name', 'version', 'dir', 'description']
        inp_list = list()
        dep_node_list = list()
        pkg_dict = dict()
        for line in self.full_ed_lines:
            inp_list.append(line.text())
        dep_pkg = inp_list[6].split(', ')
        if dep_pkg[len(dep_pkg) - 1] == '':
            dep_pkg.pop()
        # dep_node = inp_list[8].split(', ')
        # dep_node.pop()
        for dep in self.manager.wid.sub_list:
            dep_node.append(dep['msg_type'])
        for dep in self.manager.wid.pub_list:
            dep_node.append(dep['msg_type'])
        for dep in dep_node:
            msg, msg_type = dep.split('::')
            dep_node_list.append({'name': msg, 'type': msg_type})
        # for dep in dep_node:
        #     msg, msg_type = dep.split('/')
        #     dep_node_list.append({'name': msg, 'type': msg_type})
        for param, value in zip(param_list, inp_list):
            pkg_dict[param] = value
        pkg_dict['maintainer'] = {'name': inp_list[4], 'email':  inp_list[5]}
        pkg_dict['depend'] = dep_pkg
        pkg_dict['node'] = dict()
        pkg_dict['node']['name'] = inp_list[7]
        pkg_dict['node']['depend'] = dep_node_list
        pkg_dict['node']['subscribers'] = self.manager.wid.sub_list
        pkg_dict['node']['publishers'] = self.manager.wid.pub_list
        print(pkg_dict)
        return pkg_dict

    def clear_all_lines(self):
        """
        Function clear all edit lines
        """

        for line in self.full_ed_lines:
            line.setText("")
        self.msg2Statusbar.emit('Произведена полная отчистка полей')

    def generate(self):
        """
        Function run generating ROS package
        """

        if self.check_data():
            pkg_dict = self.create_package_dict()
            pkg_dir = pkg_dict['dir'] + pkg_dict['name']
            if os.path.exists(pkg_dir):
                but = QMessageBox().question(self, 'Message', "Такой проект уже существует! Хотите перезаписать?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if but == QMessageBox.Yes:
                    shutil.rmtree(pkg_dir)
                    self.pkg = package.RosPackage(pkg_dict)
                    self.msg2Statusbar.emit('Успешная генерация')
            else:
                self.pkg = package.RosPackage(pkg_dict)
                self.msg2Statusbar.emit('Успешная генерация')

    def show_manager(self):
        """
        Function show subscribers and publishers manager's window
        """

        if self.manager.wid.table.rowCount() == 0:
            self.manager.wid.add_row(0)

        self.manager.show()

    def open_fileDialog(self):
        """
        Function open dialog for import file
        """

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(self, "Открыть исходный файл", os.path.expanduser("~"),
                                      "XML Файлы (*.xml);;JSON Файлы (*.json)", options=options)
        if fileName:
            file_format = fileName.split('.')[1]
            if file_format == 'xml':
                self.data_from_xml(fileName)
            elif file_format == 'json':
                self.data_from_json(fileName)
            self.msg2Statusbar.emit('Импорт из файла {0}'.format(fileName))

    def save_fileDialog(self):
        """
        Function open dialog for save file
        """

        if self.check_data():
            options = QFileDialog.Options()
            options |= QFileDialog.DontUseNativeDialog
            fileName, _ = QFileDialog.getSaveFileName(self, "Сохранить как", os.path.expanduser("~"), "Все файлы (*);;XML Файлы (*.xml);;JSON Файлы (*.json)", options=options)
            if fileName:
                file_format = fileName.split('.')[1]
                if file_format =='xml':
                    self.create_gen_xml(fileName)
                elif file_format =='json':
                    self.create_gen_json(fileName)
                self.msg2Statusbar.emit('Сохранено в файл: {0}'.format(fileName))

    def ch_dirDialog(self):
        """
        Function open dialog for choosing directory
        """

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        dir_path = QFileDialog.getExistingDirectory(self, "Выбор папки",  os.path.expanduser("~"))
        if dir_path:
            self.full_ed_lines[2].setText(dir_path + '/')
            self.change_data()
            print(dir_path)

    def change_data(self):
        """
        Function set flag if data changed
        """

        if self.changed is not True:
            self.changed = True
            print('True')

    def change_pkg_name(self):
        """
        Function automation input for node name
        """

        sender = self.sender()
        self.change_data()
        self.full_ed_lines[7].setText(sender.text() + '_node')



if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = GenGui()
    sys.exit(app.exec_())