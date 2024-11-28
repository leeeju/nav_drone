import tempfile
from typing import Dict, List, Optional, Text

import launch
import yaml


class DictItemReference:
    """
    A helper class to manage dictionary keys and their values for updates.
    """

    def __init__(self, dictionary: dict, key: Text):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self) -> Text:
        return self.dictKey

    def setValue(self, value):
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):
    """
    A substitution that modifies the provided YAML file with parameter replacements.

    This substitution reads an existing YAML file, updates specific parameters,
    and writes the modified content into a temporary file. It supports:
    - Substituting parameters at specific paths
    - Adding new parameters if they do not exist
    - Rewriting keys in the YAML structure

    Primarily used in ROS2 launch systems.
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: Dict,
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[Dict] = None,
        convert_types: bool = False,
    ) -> None:
        """
        Initialize the RewrittenYaml substitution.

        :param source_file: The path to the original YAML file.
        :param param_rewrites: Dictionary of parameter mappings to replace.
        :param root_key: (Optional) If provided, the contents are placed under this root key.
        :param key_rewrites: (Optional) Dictionary for key replacements.
        :param convert_types: Whether to attempt converting string values to numbers or booleans.
        """
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions

        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {
            key: normalize_to_list_of_substitutions(value)
            for key, value in param_rewrites.items()
        }
        self.__key_rewrites = (
            {
                key: normalize_to_list_of_substitutions(value)
                for key, value in key_rewrites.items()
            }
            if key_rewrites
            else {}
        )
        self.__root_key = (
            normalize_to_list_of_substitutions(root_key) if root_key else None
        )
        self.__convert_types = convert_types

    @property
    def name(self) -> List[launch.Substitution]:
        """Get the source file substitutions."""
        return self.__source_file

    def describe(self) -> Text:
        """Return a string description of this substitution."""
        return f"RewrittenYaml substitution for {self.__source_file}"

    def perform(self, context: launch.LaunchContext) -> Text:
        """
        Perform the substitution, rewriting the YAML file as specified.

        Reads the source YAML file, applies parameter substitutions and key replacements,
        and writes the modified content to a temporary file.

        :param context: The launch context.
        :return: The path to the modified YAML file.
        """
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        rewritten_yaml = tempfile.NamedTemporaryFile(mode="w", delete=False)
        param_rewrites, key_rewrites = self.resolve_rewrites(context)

        with open(yaml_filename, "r") as input_file:
            data = yaml.safe_load(input_file)

        self.substitute_params(data, param_rewrites)
        self.add_params(data, param_rewrites)
        self.substitute_keys(data, key_rewrites)

        if self.__root_key:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}

        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context: launch.LaunchContext):
        """Resolve substitutions for parameters and keys in the given context."""
        resolved_params = {
            key: launch.utilities.perform_substitutions(context, value)
            for key, value in self.__param_rewrites.items()
        }
        resolved_keys = {
            key: launch.utilities.perform_substitutions(context, value)
            for key, value in self.__key_rewrites.items()
        }
        return resolved_params, resolved_keys

    def substitute_params(self, yaml_data: dict, param_rewrites: Dict):
        """
        Substitute leaf-only and absolute path parameters in the YAML.

        :param yaml_data: The YAML data as a dictionary.
        :param param_rewrites: Dictionary of parameter replacements.
        """
        for key in self.getYamlLeafKeys(yaml_data):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        yaml_paths = self.pathify(yaml_data)
        for path in yaml_paths:
            if path in param_rewrites:
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split(".")
                yaml_data = self.updateYamlPathVals(yaml_data, yaml_keys, rewrite_val)

    def add_params(self, yaml_data: dict, param_rewrites: Dict):
        """
        Add new parameters to the YAML if they do not exist.

        :param yaml_data: The YAML data as a dictionary.
        :param param_rewrites: Dictionary of parameters to add.
        """
        yaml_paths = self.pathify(yaml_data)
        for path, value in param_rewrites.items():
            if path not in yaml_paths:
                new_val = self.convert(value)
                yaml_keys = path.split(".")
                if "ros__parameters" in yaml_keys:
                    yaml_data = self.updateYamlPathVals(yaml_data, yaml_keys, new_val)

    def updateYamlPathVals(self, yaml_data: dict, yaml_key_list: List[Text], rewrite_val):
        """
        Update the value at a specific path in the YAML.

        :param yaml_data: The YAML data as a dictionary.
        :param yaml_key_list: List of keys representing the path.
        :param rewrite_val: The new value to set.
        """
        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml_data[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(yaml_data, list):
                yaml_data[int(key)] = self.updateYamlPathVals(
                    yaml_data[int(key)], yaml_key_list, rewrite_val
                )
            else:
                yaml_data[key] = self.updateYamlPathVals(
                    yaml_data.get(key, {}), yaml_key_list, rewrite_val
                )
        return yaml_data

    def substitute_keys(self, yaml_data: dict, key_rewrites: Dict):
        """Replace keys in the YAML data according to the key_rewrites dictionary."""
        for key in list(yaml_data.keys()):
            val = yaml_data[key]
            if key in key_rewrites:
                new_key = key_rewrites[key]
                yaml_data[new_key] = yaml_data.pop(key)
            if isinstance(val, dict):
                self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yaml_data):
        """Get references to all leaf keys in the YAML data."""
        try:
            for key in yaml_data.keys():
                for k in self.getYamlLeafKeys(yaml_data[key]):
                    yield k
                yield DictItemReference(yaml_data, key)
        except AttributeError:
            return

    def pathify(self, d, p=None, paths=None, joinchar="."):
        """Convert the YAML structure into a dictionary of paths."""
        if p is None:
            paths = {}
            self.pathify(d, "", paths, joinchar=joinchar)
            return paths
        pn = p + joinchar if p else ""
        if isinstance(d, dict):
            for k, v in d.items():
                self.pathify(v, f"{pn}{k}", paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, f"{pn}{idx}", paths, joinchar=joinchar)
        else:
            paths[p] = d

    def convert(self, text_value: Text):
        """Convert text to int, float, or bool if enabled; otherwise, return as string."""
        if self.__convert_types:
            try:
                return float(text_value) if "." in text_value else int(text_value)
            except ValueError:
                pass

            if text_value.lower() in {"true", "false"}:
                return text_value.lower() == "true"

        return text_value
