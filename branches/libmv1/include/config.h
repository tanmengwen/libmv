#ifndef _CONFIG_H
#define _CONFIG_H

#include <string>
#include <stdio.h>
#include "extern/json/json.h"

namespace mv {

using std::string;

class Config
{
public:
	Config() {
		js_ = json::NewMap();
		free_json_tree_on_delete_ = true;
	}

	Config(const Config &other) {
		view_tree(other.js_);
		// wow this is called a fair amount
//		printf("Called copy constructioR!!!!!\n");
	}

	Config(json::Json *subtree) {
		view_tree(subtree);
	}

	~Config() {
		if (free_json_tree_on_delete_)
			delete js_;
	}

	bool has_parameter(const string &name) {
		return js_->has_key(name);
	}

	void parameter(const string &name, double default_value, double *out) {
		set_if_missing(name, default_value);
		*out = js_->get(name).as_double();
	}

	void parameter(const string &name, int default_value, int *out) {
		set_if_missing(name, default_value);
		*out = js_->get(name).as_int();
	}

	void parameter(const string &name, const string &default_value, string *out) {
		set_if_missing(name, default_value);
		*out = js_->get(name).as_string();
	}

	void parameter(const string &name, bool default_value, bool *out) {
		if (!js_->has_key(name)) {
			if (default_value) {
				js_->set_true(name);
			} else {
				js_->set_false(name);
			}
		}
		*out = js_->get(name).as_bool();
	}

	void set_parameter(const string &name, double value) {
		js_->set(name, value);
	}

	Config subconfig(const string &name) {
		json::Json &subtree = js_->set_new_map(name);
		return Config(&subtree);
	}

	void transfer_and_release(const string &name, json::Json *dest) {
		dest->set(name, js_);
		free_json_tree_on_delete_ = false;
	}

	void set_view() {
		free_json_tree_on_delete_ = false;
	}

	json::Json* json() {
		return js_;
	}
private:
	template <typename T>
	void set_if_missing(const string &name, T value) {
		if (!js_->has_key(name))
			js_->set(name, value);
	}
	void view_tree(json::Json* js) {
		js_ = js;
		free_json_tree_on_delete_ = false;
	}
	json::Json *js_;
	bool free_json_tree_on_delete_;
};

// Shorthand notation for 'configurable classes'; the class must have a config_
// member, and the variable is declared.
#define PARAMETER(type, name, def) \
	type name; \
	config_.parameter(#name, def, &name);

Config LoadConfig(json::Json *js);
Config LoadConfig();

} // namespace mv

#endif // _CONFIG_H
