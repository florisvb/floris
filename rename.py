import os
from optparse import OptionParser


def rename(directory, prefix, removeprefix=None, fake=False):
    cmd = 'ls ' + directory
    ls = os.popen(cmd).read()
    filelist = ls.split('\n')
    try:
        filelist.remove('')
    except:
        pass
        
    for filename in filelist:
        new_filename = filename
        if removeprefix is not None:
            new_filename = new_filename.lstrip(removeprefix)
        new_filename = prefix + new_filename
        
        if fake:
            print filename, '  >>  ', new_filename 
        if not fake:
            cmd = 'mv ' + directory+'/'+filename + ' ' + directory+'/'+new_filename
            print cmd
            os.system(cmd)
        
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--d", type="str", dest="directory", default=None,
                        help="directory in which files live")
    parser.add_option("--p", type="str", dest="prefix", default=None,
                        help="prefix to add")
    parser.add_option("--rp", type="str", dest="removeprefix", default=None,
                        help="prefix to remove")
    parser.add_option("--f", action='store_true', dest="fake", default=False,
                        help="do a dry (fake) run")
                        
    (options, args) = parser.parse_args()
        
    rename( directory = options.directory,
            prefix = options.prefix, 
            removeprefix = options.removeprefix,
            fake = options.fake)

        
