/*************************************************************************/ /*
 avb-mse

 Copyright (C) 2015-2016 Renesas Electronics Corporation

 License        Dual MIT/GPLv2

 The contents of this file are subject to the MIT license as set out below.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 Alternatively, the contents of this file may be used under the terms of
 the GNU General Public License Version 2 ("GPL") in which case the provisions
 of GPL are applicable instead of those above.

 If you wish to allow use of your version of this file only under the terms of
 GPL, and not to allow others to use your version of this file under the terms
 of the MIT license, indicate your decision by deleting the provisions above
 and replace them with the notice and other provisions required by GPL as set
 out in the file called "GPL-COPYING" included in this distribution. If you do
 not delete the provisions above, a recipient may use your version of this file
 under the terms of either the MIT license or GPL.

 This License is also included in this distribution in the file called
 "MIT-COPYING".

 EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 GPLv2:
 If you wish to use this file under the terms of GPL, following terms are
 effective.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ /*************************************************************************/

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME "/" fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include "mse_sysfs.h"

#define MSE_SYSFS_SIZE	(10)
#define MSE_SYSFS_DATA	(16)

#define MSE_SYSFS_NAME_OFFSET		(3)
#define MSE_SYSFS_NAME_LENGTH		(10)
#define MSE_SYSFS_RADIX_DECIMAL		(10)
#define MSE_SYSFS_FILE_PERMISSION	(0664)
#define MSE_SYSFS_BUF_LENGTH		(255)

static struct mse_sysfs_config *config_table[MSE_SYSFS_SIZE];
static struct kobject *mse_kobj_table[MSE_SYSFS_SIZE];
static struct attribute_group *mse_group[MSE_SYSFS_SIZE];

static ssize_t mse_show(struct kobject *kobj,
			struct kobj_attribute *attr,
			char *buf)
{
	int no, i, j, ret;

	pr_debug("[%s] show %s/%s\n", __func__, kobj->name, attr->attr.name);

	ret = sscanf(kobj->name + MSE_SYSFS_NAME_OFFSET, "%d", &no);
	if (ret <= 0)
		return sprintf(buf, "error ret=%d\n", ret);
	for (i = 0; config_table[no][i].name && i < MSE_SYSFS_DATA; i++) {
		if (!strcmp(config_table[no][i].name, attr->attr.name))
			break;
	}

	switch (config_table[no][i].type) {
	case MSE_TYPE_INT:
		return sprintf(buf, "%d\n", config_table[no][i].int_value);
	case MSE_TYPE_STR:
		return sprintf(buf, "%s\n", config_table[no][i].str_value);
	case MSE_TYPE_ENUM:
		j = config_table[no][i].int_value;
		return sprintf(buf, "%s\n", config_table[no][i].enum_list[j]);
	case MSE_TYPE_MEM:
		for (j = 0; j < config_table[no][i].int_value; j++)
			buf[j] = config_table[no][i].str_value[j];
		return j;
	default:
		pr_err("[%s] unknown type=%d\n",
		       __func__, config_table[no][i].type);
	}

	return sprintf(buf, "error\n");
}

static ssize_t mse_store(struct kobject *kobj,
			 struct kobj_attribute *attr,
			 const char *buf,
			 size_t len)
{
	int no, i, value, ret;
	struct mse_sysfs_config *config;

	pr_debug("[%s]  modify %s/%s <- %s\n",
		 __func__, kobj->name, attr->attr.name, buf);

	ret = sscanf(kobj->name + MSE_SYSFS_NAME_OFFSET, "%d", &no);
	if (ret <= 0)
		return ret;
	for (i = 0; config_table[no][i].name && i < MSE_SYSFS_DATA; i++) {
		if (!strcmp(config_table[no][i].name, attr->attr.name))
			break;
	}
	config = &config_table[no][i];

	switch (config->type) {
	case MSE_TYPE_INT:
		value = kstrtoint(buf, MSE_SYSFS_RADIX_DECIMAL,
				  &config->int_value);
		break;
	case MSE_TYPE_STR:
		strncpy(config->str_value, buf, sizeof(config->str_value));
		break;
	case MSE_TYPE_ENUM:
		/* TODO : ignore spaces */
		for (value = 0; config->enum_list[value] &&
		     strncmp(config->enum_list[value], buf,
			     strlen(config->enum_list[value])) != 0;
		     value++)
			;
		if (config->enum_list[value]) {
			config->int_value = value;
		} else {
			pr_err("[%s] sysfs unknown enum value '%s'\n",
			       __func__, buf);
		}
		break;
	case MSE_TYPE_MEM:
		memcpy(config->str_value, buf, len);
		config->int_value = len;
		break;
	default:
		pr_err("[%s] unknown type=%d\n", __func__, config->type);
	}

	return len;
}

int mse_sysfs_init(struct mse_sysfs_config *config)
{
	int ret;
	int i, j;
	char config_name[MSE_SYSFS_NAME_LENGTH];
	struct kobject *mod_kobj;
	struct kobject *mse_kobj;
	struct attribute_group *mse_attr_group;
	struct attribute **mse_attrs;
	struct kobj_attribute *mse_attr;

	for (i = 0; i < ARRAY_SIZE(config_table); i++) {
		if (!config_table[i]) {
			config_table[i] = config;
			break;
		}
	}
	pr_debug("[%s] create /sys/kernel/mse%d\n", __func__, i);

	/* create /sys/kernel/mse */
	sprintf(config_name, "mse%d", i);

#ifdef MODULE
	mod_kobj = &((THIS_MODULE)->mkobj.kobj);
#else
	mod_kobj = kset_find_obj(module_kset, KBUILD_MODNAME);
#endif
	if (!mod_kobj)
		return -ENOMEM;

	mse_kobj = kobject_create_and_add(config_name, mod_kobj);
	if (!mse_kobj)
		return -ENOMEM;

	mse_attr_group = kzalloc(sizeof(*mse_attr_group), GFP_KERNEL);

	mse_attrs = kzalloc(sizeof(*mse_attrs) * MSE_SYSFS_DATA, GFP_KERNEL);

	for (j = 0; config[j].name && j < MSE_SYSFS_DATA; j++) {
		mse_attr = kzalloc(sizeof(*mse_attr), GFP_KERNEL);
		mse_attr->attr.name = config[j].name;
		mse_attr->attr.mode = MSE_SYSFS_FILE_PERMISSION;
		mse_attr->show = mse_show;
		mse_attr->store = mse_store;
		mse_attrs[j] = (struct attribute *)mse_attr;
	}
	mse_attr_group->attrs = mse_attrs;

	/* create group for pseudo device */
	ret = sysfs_create_group(mse_kobj, mse_attr_group);
	if (ret) {
		for (j = 0; mse_attrs[j]; j++)
			kfree(mse_attrs[j]);
		kfree(mse_attrs);
		kfree(mse_attr_group);
		kobject_put(mse_kobj);
		mse_kobj = NULL;
		config_table[i] = NULL;

		return -ENODEV;
	}

	mse_group[i] = mse_attr_group;
	mse_kobj_table[i] = mse_kobj;

	return i;
}

void mse_sysfs_exit(int index)
{
	int i;

	if (index >= 0 && index < ARRAY_SIZE(mse_kobj_table)) {
		sysfs_remove_group(mse_kobj_table[index], mse_group[index]);
		for (i = 0; mse_group[index]->attrs[i]; i++)
			kfree(mse_group[index]->attrs[i]);

		kfree(mse_group[index]->attrs);
		kfree(mse_group[index]);
		mse_group[index] = NULL;
		kobject_put(mse_kobj_table[index]);
		mse_kobj_table[index] = NULL;
		config_table[index] = NULL;
	}
}

static struct mse_sysfs_config *search_config(int index, char *name)
{
	int attr;
	struct mse_sysfs_config *config = config_table[index];

	for (attr = 0; config[attr].name; attr++)
		if (!strcmp(config[attr].name, name))
			return &config[attr];

	return NULL;
}

int mse_sysfs_get_config_int(int index, char *name, int *value)
{
	struct mse_sysfs_config *config = search_config(index, name);

	if (!config)
		return -ENOENT;

	switch (config->type) {
	case MSE_TYPE_INT:
	case MSE_TYPE_ENUM:
		*value = config->int_value;
		pr_debug("[%s] %s=%d\n", __func__, name, *value);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int mse_sysfs_get_config_str(int index, char *name, char *buf, int buf_size)
{
	struct mse_sysfs_config *config = search_config(index, name);
	int i, j;
	char work_buf[MSE_SYSFS_BUF_LENGTH];

	if (!config)
		return -ENOENT;

	switch (config->type) {
	case MSE_TYPE_STR:
		strncpy(buf, config->str_value, buf_size);
		pr_debug("[%s] %s=%s\n", __func__, name, buf);
		break;
	case MSE_TYPE_MEM:
		memcpy(buf, config->str_value, buf_size);
		for (i = 0, j = 0; i < buf_size; i++)
			j += sprintf(&work_buf[j], "%02x", buf[i]);
		pr_debug("[%s] %s=0x%s\n", __func__, name, work_buf);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int mse_sysfs_set_config_int(int index, char *name, int value)
{
	struct mse_sysfs_config *config = search_config(index, name);

	if (!config)
		return -ENOENT;

	switch (config->type) {
	case MSE_TYPE_INT:
	case MSE_TYPE_ENUM:
		config->int_value = value;
		pr_debug("[%s] %s=%d\n", __func__, name, value);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int mse_sysfs_set_config_str(int index, char *name, char *buf, int buf_size)
{
	struct mse_sysfs_config *config = search_config(index, name);
	int i, j;
	char work_buf[MSE_SYSFS_BUF_LENGTH];

	if (!config)
		return -ENOENT;

	switch (config->type) {
	case MSE_TYPE_STR:
		strncpy(config->str_value, buf, sizeof(config->str_value));
		pr_debug("[%s] %s=%s\n", __func__, name, buf);
		break;
	case MSE_TYPE_MEM:
		memcpy(config->str_value, buf, buf_size);
		config->int_value = buf_size;
		for (i = 0, j = 0; i < buf_size; i++) {
			j += sprintf(&work_buf[j], "%02x",
				     config->str_value[i]);
		}
		pr_debug("[%s] %s=0x%s\n", __func__, name, work_buf);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
