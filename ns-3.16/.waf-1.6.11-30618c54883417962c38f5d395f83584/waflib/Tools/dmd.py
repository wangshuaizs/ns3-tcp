#! /usr/bin/env python
# encoding: utf-8
# WARNING! Do not edit! http://waf.googlecode.com/git/docs/wafbook/single.html#_obtaining_the_waf_file

import sys
from waflib.Tools import ar,d
from waflib.Configure import conf
def find_dmd(conf):
	conf.find_program(['dmd','ldc'],var='D')
def common_flags_ldc(conf):
	v=conf.env
	v['DFLAGS']=['-d-version=Posix']
	v['LINKFLAGS']=[]
	v['DFLAGS_dshlib']=['-relocation-model=pic']
def common_flags_dmd(conf):
	v=conf.env
	v['D_SRC_F']=['-c']
	v['D_TGT_F']='-of%s'
	v['D_LINKER']=v['D']
	v['DLNK_SRC_F']=''
	v['DLNK_TGT_F']='-of%s'
	v['DINC_ST']='-I%s'
	v['DSHLIB_MARKER']=v['DSTLIB_MARKER']=''
	v['DSTLIB_ST']=v['DSHLIB_ST']='-L-l%s'
	v['DSTLIBPATH_ST']=v['DLIBPATH_ST']='-L-L%s'
	v['LINKFLAGS_dprogram']=['-quiet']
	v['DFLAGS_dshlib']=['-fPIC']
	v['LINKFLAGS_dshlib']=['-L-shared']
	v['DHEADER_ext']='.di'
	v.DFLAGS_d_with_header=['-H','-Hf']
	v['D_HDR_F']='%s'
def configure(conf):
	conf.find_dmd()
	if sys.platform=='win32':
		out=conf.cmd_and_log([conf.env.D,'--help'])
		if out.find("D Compiler v2.")>-1:
			conf.fatal('dmd2 on Windows is not supported, use gdc or ldc instead')
	conf.load('ar')
	conf.load('d')
	conf.common_flags_dmd()
	conf.d_platform_flags()
	if str(conf.env.D).find('ldc')>-1:
		conf.common_flags_ldc()

conf(find_dmd)
conf(common_flags_ldc)
conf(common_flags_dmd)