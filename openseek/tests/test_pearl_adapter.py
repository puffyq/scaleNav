import unittest

from text_tracker.pearl_adapter import runtime_classes


class RuntimeClassesTest(unittest.TestCase):
    def test_target_is_not_duplicated_in_competing_classes(self) -> None:
        classes, target_index = runtime_classes("tree")

        self.assertEqual(classes[target_index], "tree")
        competitor_aliases = {
            alias.casefold()
            for class_index, class_names in enumerate(classes)
            if class_index != target_index
            for alias in class_names.split(", ")
        }
        self.assertNotIn("tree", competitor_aliases)
        self.assertIn("road", competitor_aliases)

    def test_single_alias_competitor_is_removed_for_runtime_target(self) -> None:
        classes, target_index = runtime_classes("car")

        self.assertEqual(classes[target_index], "car")
        self.assertEqual(
            sum(alias.casefold() == "car" for names in classes for alias in names.split(", ")),
            1,
        )

    def test_prompt_must_not_be_empty(self) -> None:
        with self.assertRaisesRegex(ValueError, "cannot be empty"):
            runtime_classes("  ")


if __name__ == "__main__":
    unittest.main()
