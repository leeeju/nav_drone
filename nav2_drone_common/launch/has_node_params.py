from typing import List, Text
import launch
import yaml


class HasNodeParams(launch.Substitution):
    """
    A substitution that checks if a parameter file contains parameters for a specific node.

    This is used in the launch system to dynamically evaluate whether a node's parameters
    are present in a YAML file.
    """

    def __init__(self, source_file: launch.SomeSubstitutionsType, node_name: Text) -> None:
        """
        Construct the substitution.

        :param source_file: The path to the parameter YAML file, as a Launch substitution.
        :param node_name: The name of the node to check for in the parameter file.
        """
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions  # Lazy import to avoid circular dependencies

        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__node_name = node_name

    @property
    def source_file(self) -> List[launch.Substitution]:
        """Getter for the parameter YAML file."""
        return self.__source_file

    def describe(self) -> Text:
        """
        Return a description of this substitution.

        This can be useful for debugging or logging the state of the substitution.
        """
        return f"Check if node '{self.__node_name}' exists in parameter file."

    def perform(self, context: launch.LaunchContext) -> Text:
        """
        Perform the substitution to check if the node exists in the parameter file.

        :param context: The launch context in which substitutions are evaluated.
        :return: 'True' if the node exists in the YAML file, otherwise 'False'.
        """
        try:
            # Resolve the file path using Launch substitutions
            yaml_filename = launch.utilities.perform_substitutions(context, self.source_file)

            # Open and load the YAML file
            with open(yaml_filename, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file)

            # Check if the node name is a key in the YAML file
            if isinstance(data, dict) and self.__node_name in data:
                return 'True'
            return 'False'

        except FileNotFoundError as e:
            raise RuntimeError(f"Parameter file '{yaml_filename}' not found: {e}") from e
        except yaml.YAMLError as e:
            raise RuntimeError(f"Error parsing YAML file '{yaml_filename}': {e}") from e
        except Exception as e:
            raise RuntimeError(f"Unexpected error while checking node parameters: {e}") from e
