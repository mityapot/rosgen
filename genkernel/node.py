class RosNode:
    """
    RosNode class, generate node files in  ROS package
    """

    depend = list()
    subscribers = list()
    publishers = list()
    template_names = dict()
    src_dir = None

    def __init__(self, pkg_dict):
        """
        RosNode object constructor
        :param pkg_dict: package parameters in special format
        :type pkg_dict: dict
        """

        self.pkg_dict = pkg_dict
        for depend in self.pkg_dict['node']['depend']:
            str_depend = depend['name'] + '/' + depend['type']
            self.depend.append(str_depend)
        self.subscribers = self.pkg_dict['node']['subscribers']
        self.publishers = self.pkg_dict['node']['publishers']
        self.name = self.pkg_dict['node']['name']

    def generate_node_code(self, node_dir):
        """
        Function generate all source  C++ code of node
        :param node_dir: path to source node code
        :type: str
        :return: topics for readme in special format
        :rtype: dict
        """

        self.src_dir = node_dir
        self.make_node_interface()
        self.make_node_realisation()
        self.make_main()
        sub_str_list = list()
        pub_str_list = list()
        for subscriber in self.subscribers:
            sub_str = '{} - {} topic, `{}`'.format(subscriber['topic_name'], subscriber['name'].split('_')[0], subscriber['msg_type'])
            sub_str_list.append(sub_str)
        for publisher in self.publishers:
            pub_str = '{} - {} topic, `{}`'.format(publisher['topic_name'], publisher['name'].split('_')[0], publisher['msg_type'])
            pub_str_list.append(pub_str)
        out_topics = {'subscribed': sub_str_list, 'published': pub_str_list}
        return out_topics

    def make_node_interface(self):
        """
        Function generate node interface code
        """

        name_list = self.name.split('_')
        self.template_names['flname'] = name_list[0] + '_' + name_list[1]
        self.template_names['classname'] = name_list[0].capitalize() + name_list[1].capitalize()
        self.template_names['preproc_dir'] = name_list[0].upper() + '_' + name_list[1].upper() + '_HPP'

        f = open('../genkernel/templates/node.hpp')
        o = open(self.src_dir + '/' + self.template_names['flname'] + '.hpp', 'a')
        dep_flag = 0
        block_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_dict['name'])
            line = line.replace('[class_name]', self.template_names['classname'])
            line = line.replace('[date]', self.pkg_dict['str_date'])
            line = line.replace('[author]', self.pkg_dict['maintainer']['name'])
            line = line.replace('[preproc_dir]', self.template_names['preproc_dir'])
            if line.find('<ros/ros.h>') != -1 and dep_flag != -1:
                dep_flag = 1
            if line.find('ros::NodeHandle _node;') != -1 and block_flag != -1:
                block_flag = 1
            o.write(line)
            if dep_flag == 1:
                for depend in self.depend:
                    o.write('#include <{}.h>\n'.format(depend))
                dep_flag = -1
            if block_flag == 1:
                for subscriber in self.subscribers:
                    o.write('  ros::Subscriber _{};\n'.format(subscriber['name']))
                for publisher in self.publishers:
                    o.write('  ros::Publisher _{};\n'.format(publisher['name']))
                o.write('\n')
                for subscriber in self.subscribers:
                    o.write('  void {}Callback(const {}& message);\n'.format(subscriber['name'].split('_')[0], subscriber['msg_type']))
                block_flag = -1
        o.close()
        f.close()

    def make_node_realisation(self):
        """
        Function generate node realisation code
        """

        f = open('../genkernel/templates/node.cpp')
        o = open(self.src_dir + '/' + self.template_names['flname'] + '.cpp', 'a')
        sub_flag = 0
        pub_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_dict['name'])
            line = line.replace('[class_name]', self.template_names['classname'])
            line = line.replace('[date]', self.pkg_dict['str_date'])
            line = line.replace('[author]', self.pkg_dict['maintainer']['name'])
            line = line.replace('[flname]', self.template_names['flname'])
            if line.find('// subscribers settings') != -1 and sub_flag != -1:
                sub_flag = 1
            if line.find('// publishers settings') != -1 and pub_flag != -1:
                pub_flag = 1
            o.write(line)
            if sub_flag == 1:
                for subscriber in self.subscribers:
                    o.write('  _{} = _node.subscribe(\'{}\', {}, &{}::{}Callback, this);\n'.format(subscriber['name'], subscriber['topic_name'], subscriber['queue_size'], self.template_names['classname'], subscriber['name'].split('_')[0]))
                sub_flag = -1
            if pub_flag == 1:
                for publisher in self.publishers:
                    o.write('  _{} = _node.advertise<{}>(\'{}\', {});\n'.format(publisher['name'], publisher['msg_type'], publisher['topic_name'], publisher['queue_size']))
                pub_flag = -1
        f.close()
        o.write('\n')
        for subscriber in self.subscribers:
            o.write('void {}::{}Callback(const {}& m) '.format(self.template_names['classname'], subscriber['name'].split('_')[0], subscriber['msg_type']))
            o.write('{\n}\n')
        o.close()

    def make_main(self):
        """
        Function generate main file of node
        """

        f = open('../genkernel/templates/node_main.cpp')
        o = open(self.src_dir + '/' + self.template_names['flname'] + '_main.cpp', 'a')
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_dict['name'])
            line = line.replace('[class_name]', self.template_names['classname'])
            line = line.replace('[date]', self.pkg_dict['str_date'])
            line = line.replace('[author]', self.pkg_dict['maintainer']['name'])
            line = line.replace('[flname]', self.template_names['flname'])
            line = line.replace('[node_name]', self.name)
            o.write(line)
        o.close()
        f.close()