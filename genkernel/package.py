import os
import datetime
from genkernel.node import RosNode


class RosPackage:
    """
    RosPackage class, generate ROS package in workspace directory
    """

    depend = list()
    dir_list = list()

    def __init__(self, pkg_dict):
        """
        RosPackage object constructor
        :param pkg_dict: package parameters in special format
        :type pkg_dict: dict
        """

        self.create_dt = datetime.datetime.now()
        self.pkg_dict = pkg_dict
        self.make_dirs()
        self.make_xml()
        self.make_cmake()
        self.pkg_dict['str_date'] = self.create_dt.strftime("%B %d, %Y")
        self.node = RosNode(self.pkg_dict)
        out_topics = self.node.generate_node_code(self.dir_list[1])
        self.make_launch()
        self.make_readme(out_topics)

    def make_dirs(self):
        """
        Function make all directories of  ROS package
        """

        pkg_name = self.pkg_dict['name']
        catkin_dir = self.pkg_dict['dir']
        self.dir_list = [pkg_name, pkg_name + '/src', pkg_name + '/launch', pkg_name + '/cfg', pkg_name + '/include']
        self.dir_list = [catkin_dir + pkg_dir for pkg_dir in self.dir_list]
        for path in self.dir_list:
            try:
                os.mkdir(path)
            except OSError:
                print("Создать директорию %s не удалось" % path)
            else:
                print("Успешно создана директория %s " % path)

    def make_xml(self):
        """
        Function make xml file of a package
        """

        param_list = ['name', 'version', 'description']
        f = open('../genkernel/templates/package.xml')
        o = open(self.dir_list[0] + '/package.xml', 'a')
        while 1:
            line = f.readline()
            if not line: break
            for i in range(3):
                line = line.replace('[{0}]'.format(i), self.pkg_dict[param_list[i]])
            if line.find('[3]') != -1:
                o.write('  <maintainer email="{1}">{0}</maintainer>\n'.format(self.pkg_dict['maintainer']['name'], self.pkg_dict['maintainer']['email']))
            elif line.find('[4]') != -1:
                for depend in self.pkg_dict['depend']:
                    o.write('  <build_depend>{0}</build_depend>\n'.format(depend))
            elif line.find('[5]') != -1:
                for depend in self.pkg_dict['depend']:
                    o.write('  <exec_depend>{0}</exec_depend>\n'.format(depend))
            else:
                o.write(line)
        o.close()
        f.close()

    def make_cmake(self):
        """
        Function make cmake of ROS package
        """

        pkg_name = self.pkg_dict['name']
        f = open('../genkernel/templates/CMakeLists.txt')
        o = open(self.dir_list[0] + '/CMakeLists.txt', 'a')
        dep_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', pkg_name)
            if line.find('roscpp') != -1 and dep_flag != -1:
                dep_flag = 1
            o.write(line)
            if dep_flag == 1:
                for depend in self.pkg_dict['depend']:
                    o.write("  " + depend + '\n')
                dep_flag = -1
        o.close()
        f.close()

    def make_launch(self):
        """
        Function make launch file for node
        """

        pkg_name = self.pkg_dict['name']
        f = open('../genkernel/templates/node.launch')
        o = open(self.dir_list[2] + '/node.launch', 'a')
        while 1:
            line = f.readline()
            if not line: break
            if line.find('[0]') != -1:
                o.write("  <node pkg=\"{0}\" name=\"{1}\" type=\"{2}\" args=\"\" respawn=\"true\">\n".format(pkg_name, self.pkg_dict['node']['name'], self.pkg_dict['node']['name']))
            else:
                o.write(line)
        o.close()
        f.close()

    def make_readme(self, out_topics):
        """
        Function make file about ROS package - README.md
        :param out_topics: dict with strings in special format about subscribed and published topics
        :type out_topics: dict
        """

        pkg_name = self.pkg_dict['name']
        f = open('../genkernel/templates/README.md')
        o = open(self.dir_list[0] + '/README.md', 'a')
        dep_flag = 0
        sub_flag = 0
        pub_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', pkg_name)
            line = line.replace('[author]', self.pkg_dict['maintainer']['name'])
            if line.find('[roscpp](http://wiki.ros.org/roscpp)') != -1 and dep_flag != -1:
                dep_flag = 1
            if line.find('Subscribed topics:') != -1 and sub_flag != -1:
                sub_flag = 1
            if line.find('Published topics:') != -1 and pub_flag != -1:
                pub_flag = 1
            o.write(line)
            if dep_flag == 1:
                for depend in self.pkg_dict['depend']:
                    o.write('* {}\n'.format(depend))
                dep_flag = -1
            if sub_flag == 1:
                for sub_str in out_topics['subscribed']:
                    o.write('* {}\n'.format(sub_str))
                sub_flag = -1
            if pub_flag == 1:
                for pub_str in out_topics['published']:
                    o.write('* {}\n'.format(pub_str))
                pub_flag = -1
        o.close()
        f.close()


