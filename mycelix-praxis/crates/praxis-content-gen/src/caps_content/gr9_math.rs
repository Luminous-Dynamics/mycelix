// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Grade 9 Mathematics topic content — foundational skills for FET Phase.

use super::TopicContent;

macro_rules! stub_topic {
    ($name:ident, $title:expr, $expl:expr, $vocab_term:expr) => {
        pub(crate) struct $name;
        impl TopicContent for $name {
            fn explanation(&self) -> String { $expl.to_string() }
            fn worked_example(&self, _index: usize) -> String {
                format!("Solve a basic {} problem.\nStep 1: Identify what is given -> read the question\nStep 2: Apply the method -> work through\nStep 3: Check your answer -> verify\nAnswer: See worked solution above", $title)
            }
            fn practice_problem(&self, _difficulty: u16) -> String {
                format!("Practice {} problem.\nAnswer\nExplanation of the answer.\nDistractor 1,Distractor 2,Distractor 3\nHint: Think about the key concept.,Hint: Apply the method step by step.", $title)
            }
            fn hint(&self, level: u8) -> String {
                match level {
                    1 => format!("What is the key concept in {}?", $title),
                    2 => format!("Remember the method for {}. Apply it step by step.", $title),
                    _ => format!("For {}: start by identifying what you know, then apply the formula.", $title),
                }
            }
            fn misconception(&self) -> String {
                format!("WRONG: A common mistake in {}.\nRIGHT: The correct approach.\nWHY: Understanding why students make this error helps avoid it.", $title)
            }
            fn vocabulary(&self) -> String { format!("{}: A key term in this topic | Used in Grade 9 mathematics.", $vocab_term) }
            fn flashcard(&self) -> String { format!("What is the key concept in {}? | {}", $title, $expl) }
            fn assessment_item(&self, _context: &str) -> String {
                format!("Explain the key idea behind {}.\nThe main concept is...\n3\nThink about what you learned.", $title)
            }
        }
    };
}

stub_topic!(
    Gr9WholeNumbersTopic,
    "Whole Numbers and Integers",
    "In Grade 9, you work with integers — positive and negative whole numbers. BODMAS (Brackets, Orders, Division, Multiplication, Addition, Subtraction) tells you the order to do calculations. When multiplying integers: positive × positive = positive, negative × negative = positive, positive × negative = negative. You also learn prime factorisation, HCF (highest common factor) and LCM (lowest common multiple).",
    "integer"
);

stub_topic!(
    Gr9FractionsDecimalsTopic,
    "Fractions, Decimals, and Percentages",
    "Grade 9 covers converting between fractions, decimals, and percentages. To convert a fraction to a decimal, divide the numerator by the denominator. To convert to a percentage, multiply by 100. Ratio and proportion are used to compare quantities and solve problems like sharing amounts or scaling recipes.",
    "ratio"
);

stub_topic!(
    Gr9ExponentsTopic,
    "Exponents",
    "Exponent laws let you simplify expressions with powers. Key rules: a^m × a^n = a^(m+n), a^m ÷ a^n = a^(m-n), (a^m)^n = a^(mn), (ab)^n = a^n × b^n, a^0 = 1 (for a ≠ 0). Scientific notation writes very large or small numbers as a number between 1 and 10 multiplied by a power of 10.",
    "exponent"
);

pub(crate) struct Gr9AlgebraExpressionsTopic;

impl TopicContent for Gr9AlgebraExpressionsTopic {
    fn explanation(&self) -> String {
        "An algebraic expression combines numbers, variables and operations. Like terms have the same variable part and exponent, so their coefficients can be added: 3x + 5x = 8x. Unlike terms such as 3x and 5y cannot be combined. To expand brackets, use the distributive law and multiply the outside factor by every term inside: 4(x + 3) = 4x + 12. To factorise, reverse expansion by taking out a common factor: 6x + 9 = 3(2x + 3). Keep the variable part unchanged when adding like terms, and check factorisation by expanding the result.".to_string()
    }

    fn worked_example(&self, index: usize) -> String {
        match index {
            0 => "Simplify 3x + 5x.\nIdentify the like terms -> both terms contain x to power 1\nAdd the coefficients -> 3 + 5 = 8\nKeep the variable part -> 8x\nAnswer: 3x + 5x = 8x".to_string(),
            1 => "Expand 4(x + 3).\nApply the distributive law -> 4(x + 3) = 4x + 4(3)\nMultiply each term -> 4x + 12\nCheck by reversing the distribution -> factoring 4 from 4x + 12 gives 4(x + 3)\nAnswer: 4(x + 3) = 4x + 12".to_string(),
            _ => "Factorise 6x + 9.\nFind the highest common factor -> both 6x and 9 are divisible by 3\nDivide each term by 3 -> 6x / 3 = 2x and 9 / 3 = 3\nWrite the common factor outside the brackets -> 3(2x + 3)\nAnswer: 6x + 9 = 3(2x + 3)".to_string(),
        }
    }

    fn practice_problem(&self, difficulty: u16) -> String {
        match difficulty {
            0..=200 => "Simplify 2x + 3x.\n5x\nBoth terms are like terms, so add the coefficients 2 + 3 and keep x.\n2x, 8x, 5x^2\nHint: Check whether both terms have the same variable part.\nHint: Add the coefficients; do not multiply the x symbols.".to_string(),
            201..=300 => "Expand 3(x + 2).\n3x + 6\nMultiply 3 by both terms inside the brackets: 3 times x and 3 times 2.\n3x + 2, 3x + 5, x + 6\nHint: The outside factor must multiply every term inside the brackets.\nHint: Calculate 3 times x and 3 times 2 separately.".to_string(),
            301..=400 => "Simplify 4a + 3a - a.\n6a\nCombine the coefficients: 4 + 3 - 1 = 6, then keep the variable a.\n6, 8a, 6a^2\nHint: The terms all contain the same variable a.\nHint: Work out 4 + 3 - 1.".to_string(),
            401..=550 => "Simplify 2(x + 5) + 3x.\n5x + 10\nFirst expand to 2x + 10 + 3x, then combine 2x and 3x to get 5x + 10.\n5x + 5, 5x + 12, 5x + 15\nHint: Expand the brackets before collecting like terms.\nHint: Combine the x terms and leave the constant 10.".to_string(),
            551..=650 => "Factorise 8x + 12 using the highest common factor.\n4(2x + 3)\nThe highest common factor is 4; 8x divided by 4 is 2x and 12 divided by 4 is 3.\n2(4x + 4), 4(2x + 2), 8(x + 12)\nHint: Find the greatest number that divides both 8 and 12.\nHint: Divide every term by that factor and place it outside the brackets.".to_string(),
            651..=800 => "Solve 3x + 5 = 20.\nx = 5\nSubtract 5 from both sides to get 3x = 15, then divide both sides by 3.\n3, 4, 25\nHint: Undo addition before undoing multiplication.\nHint: Keep both sides balanced when applying inverse operations.".to_string(),
            _ => "Simplify 2(3x + 4) - x.\n5x + 8\nExpand to 6x + 8 - x, then combine 6x - x to get 5x + 8.\n5x + 7, 6x + 8, 5x + 4\nHint: Apply the distributive law to both terms in the brackets.\nHint: After expansion, combine only terms with the same variable part.".to_string(),
        }
    }

    fn hint(&self, level: u8) -> String {
        match level {
            1 => "First check whether the terms have matching variables and exponents.".to_string(),
            2 => "For like terms, combine the number coefficients but keep the shared variable part.".to_string(),
            _ => "For brackets, multiply the outside factor by every term inside; then combine like terms and check by reversing the steps.".to_string(),
        }
    }

    fn misconception(&self) -> String {
        "WRONG: The terms 3x and 4y can be combined to make 7xy.\nRIGHT: They are unlike terms because their variable parts differ, so 3x + 4y cannot be simplified to one like term.\nWHY: Students may focus on adding the coefficients and overlook that the variables determine whether terms are alike.".to_string()
    }

    fn vocabulary(&self) -> String {
        "like term: A term with the same variable part and exponents as another term | 3x and 5x are like terms.\ncoefficient: The numerical factor multiplying a variable | In 7x, the coefficient is 7.\ndistributive law: Multiply a factor by each term inside brackets | 3(x + 2) = 3x + 6.\nfactorise: Rewrite an expression as a product of factors | 6x + 9 = 3(2x + 3).".to_string()
    }

    fn flashcard(&self) -> String {
        "What is 3x + 5x? | 8x, because the terms are like terms and their coefficients add."
            .to_string()
    }

    fn assessment_item(&self, context: &str) -> String {
        if context.contains("Remember") {
            "What is a coefficient in an algebraic term?\nThe numerical factor multiplying the variable, such as 7 in 7x.\nRecall the number multiplying the variable.".to_string()
        } else if context.contains("Understand") {
            "Why can 3x + 4y not be simplified to 7xy?\nThe variable parts differ, so they are unlike terms and cannot be combined.\nCompare the variable parts, not only the coefficients.".to_string()
        } else if context.contains("Apply") {
            "Expand 5(x + 2).\n5x + 10\nMultiply 5 by both x and 2.".to_string()
        } else if context.contains("Analyze") {
            "A learner writes 2(x + 4) = 2x + 4. Identify the error and correct it.\nThe factor 2 must multiply both terms: 2(x + 4) = 2x + 8.\nCheck whether the outside factor was applied to every term.".to_string()
        } else if context.contains("Evaluate") {
            "Which expression is equivalent to 6x + 12: 6(x + 2) or 6x + 2? Explain.\n6(x + 2) is equivalent because expanding gives 6x + 12; 6x + 2 is not equivalent.\nExpand the bracketed expression to compare the results.".to_string()
        } else {
            "Create a factorised expression that expands to 8x + 20, and verify it.\nFor example, 4(2x + 5) expands to 8x + 20.\nFind a common factor and check by expanding your expression.".to_string()
        }
    }
}

stub_topic!(
    Gr9AlgebraEquationsTopic,
    "Algebraic Equations",
    "Solving equations means finding the value of the variable that makes the equation true. For linear equations, use inverse operations to isolate the variable: if 3x + 7 = 22, subtract 7 from both sides (3x = 15), then divide by 3 (x = 5). Always do the same operation to both sides. Word problems: translate the English into an equation, then solve.",
    "equation"
);

stub_topic!(
    Gr9PatternsTopic,
    "Numeric and Geometric Patterns",
    "A pattern is a sequence of numbers or shapes that follows a rule. In a linear pattern, the difference between consecutive terms is constant. The general term (nth term) of a linear sequence is Tn = a + (n-1)d, where a is the first term and d is the common difference. You can use the general term to find any term without listing them all.",
    "sequence"
);

stub_topic!(
    Gr9FunctionsTopic,
    "Functions and Relationships",
    "A function is a rule that assigns exactly one output to each input. You can represent functions as equations (y = 2x + 3), tables of values, or graphs on the Cartesian plane. The Cartesian plane has an x-axis (horizontal) and y-axis (vertical). Each point is written as (x, y). Linear functions produce straight-line graphs.",
    "function"
);

stub_topic!(
    Gr9GeomStraightLinesToopic,
    "Geometry of Straight Lines",
    "Angles on a straight line add up to 180° (supplementary). Vertically opposite angles are equal. When a transversal crosses parallel lines: corresponding angles are equal (F-pattern), alternate angles are equal (Z-pattern), and co-interior angles add to 180° (C-pattern or U-pattern).",
    "transversal"
);

stub_topic!(
    Gr9Geom2DShapesTopic,
    "Geometry of 2D Shapes",
    "The angles of any triangle add up to 180°. The Theorem of Pythagoras states that in a right-angled triangle, a² + b² = c² where c is the hypotenuse (longest side, opposite the right angle). Triangles are congruent (identical) if SSS, SAS, ASA, or RHS conditions are met. Similar triangles have the same shape but different sizes.",
    "Pythagoras"
);

stub_topic!(
    Gr9MeasurementTopic,
    "Area, Perimeter, and Volume",
    "Perimeter is the distance around a shape. Area is the space inside a 2D shape: rectangle = l × w, triangle = ½ × b × h, circle = πr². Volume is the space inside a 3D shape: rectangular prism = l × w × h, cylinder = πr²h. Surface area is the total area of all faces. Always include the correct units: cm for length, cm² for area, cm³ for volume.",
    "volume"
);

stub_topic!(
    Gr9DataHandlingTopic,
    "Data Handling",
    "The mean is the sum of all values divided by the number of values. The median is the middle value when data is arranged in order. The mode is the most frequent value. The range is the difference between the highest and lowest values. Data can be displayed using bar graphs, histograms, pie charts, and line graphs.",
    "mean"
);

stub_topic!(
    Gr9ProbabilityTopic,
    "Probability",
    "Probability measures how likely an event is to occur, on a scale from 0 (impossible) to 1 (certain). P(event) = number of favourable outcomes / total number of outcomes. The probability of an event NOT happening is 1 - P(event). Tree diagrams show all possible outcomes for two or more events and help calculate combined probabilities.",
    "probability"
);

#[cfg(test)]
mod algebra_distractor_tests {
    use super::*;

    #[test]
    fn algebra_practice_distractors_do_not_repeat_the_key_across_difficulty_bands() {
        let topic = Gr9AlgebraExpressionsTopic;
        for difficulty in [100, 250, 350, 450, 600, 700, 900] {
            let item = topic.practice_problem(difficulty);
            let lines = item.lines().collect::<Vec<_>>();
            assert!(lines.len() >= 4, "practice item is incomplete at difficulty {difficulty}");
            let answer = lines[1].trim().to_lowercase();
            let choices = lines[3]
                .split(',')
                .map(str::trim)
                .map(str::to_lowercase)
                .collect::<Vec<_>>();
            assert_eq!(
                choices.len(),
                3,
                "expected three choices at difficulty {difficulty}: {item}"
            );
            assert!(
                choices.iter().all(|choice| choice != &answer),
                "the correct answer was repeated as a distractor at difficulty {difficulty}: {item}"
            );
        }

        let factorisation = topic.practice_problem(600);
        let choices = factorisation.lines().nth(3).unwrap_or_default();
        assert!(
            !choices.contains("2(4x + 6)"),
            "an algebraically equivalent factorisation must not appear as a distractor"
        );
    }
}

