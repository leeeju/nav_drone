import tempfile
from typing import Dict, List, Optional, Text

import launch


class ReplaceString(launch.Substitution):
    """
    Substitution that replaces strings in a given file.

    This utility is intended for use within a launch system, where it reads an input file,
    replaces specific strings based on a dictionary of replacements, and outputs a temporary
    file with the modifications.
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        replacements: Dict[Text, Text],
        condition: Optional[launch.Condition] = None,
    ) -> None:
        """
        Initialize the ReplaceString substitution.

        :param source_file: The input file path to process.
        :param replacements: A dictionary of strings to replace in the input file.
        :param condition: (Optional) A condition to determine if the replacement should be applied.
        """
        super().__init__()
        from launch.utilities import normalize_to_list_of_substitutions  # Avoid circular import
        self.__source_file = normalize_to_list_of_substitutions(source_file)
        self.__replacements = {
            key: normalize_to_list_of_substitutions(value) for key, value in replacements.items()
        }
        self.__condition = condition

    @property
    def name(self) -> List[launch.Substitution]:
        """Getter for the source file substitutions."""
        return self.__source_file

    @property
    def condition(self) -> Optional[launch.Condition]:
        """Getter for the condition."""
        return self.__condition

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return f"ReplaceString substitution for {self.__source_file}"

    def perform(self, context: launch.LaunchContext) -> Text:
        """
        Perform the substitution by replacing strings in the file.

        If the condition is not met, returns the original file path. Otherwise, returns
        the path to a temporary file with the substitutions applied.
        """
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)

        if self.__condition is None or self.__condition.evaluate(context):
            # Create a temporary output file
            with tempfile.NamedTemporaryFile(mode='w', delete=False) as output_file:
                replacements = self.resolve_replacements(context)
                try:
                    with open(yaml_filename, 'r') as input_file:
                        self.replace(input_file, output_file, replacements)
                except Exception as err:
                    raise RuntimeError(f"ReplaceString substitution error: {err}") from err
                return output_file.name
        else:
            return yaml_filename

    def resolve_replacements(self, context: launch.LaunchContext) -> Dict[Text, Text]:
        """
        Resolve substitutions for the replacement dictionary.

        :param context: The launch context used for resolving substitutions.
        :return: A dictionary of resolved replacements.
        """
        resolved_replacements = {
            key: launch.utilities.perform_substitutions(context, value)
            for key, value in self.__replacements.items()
        }
        return resolved_replacements

    def replace(self, input_file, output_file, replacements: Dict[Text, Text]) -> None:
        """
        Perform string replacements line-by-line in the input file and write to the output file.

        :param input_file: The input file object.
        :param output_file: The output file object.
        :param replacements: A dictionary of string replacements to apply.
        """
        for line in input_file:
            for key, value in replacements.items():
                if not isinstance(key, str) or not isinstance(value, str):
                    raise TypeError(
                        f"Invalid replacement pair: key='{key}', value='{value}'. Both must be strings."
                    )
                line = line.replace(key, value)
            output_file.write(line)
