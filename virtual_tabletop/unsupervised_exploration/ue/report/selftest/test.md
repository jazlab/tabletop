# Self-Test Report: md2pdf Features

This paragraph has **bold**, *italic*, ***bold italic***, `inline code`,
a [link to example.com](https://example.com/?a=1&b=2), and special chars:
5 < 6 & 7 > 3, <tag> stays literal. Math: $\alpha_t = \beta^2 + γ$ and
unicode $θ ∈ [0, π]$; dollars like $5 and $10 stay put. snake_case_name
is not italic. Escaped \*stars\*. Internal link to [Tables](#tables).

## Lists

- First bullet with **bold**
- Second bullet that is quite long so that it wraps over more than one line
  of text in the rendered PDF output, testing hanging indentation.
  - Nested bullet A
  - Nested bullet B with `code`
- Third bullet

1. Step one
2. Step two
   1. Sub-step a
   2. Sub-step b
3. Step three

- [x] Done task
- [ ] Open task

## Tables

| Model | Params | Accuracy | Notes |
|:------|-------:|:--------:|-------|
| Baseline | 1.2M | 0.81 | Simple MLP |
| **Ours** | 3.4M | 0.93 | Uses $\lambda = 0.1$ and a very long note that should wrap inside the cell nicely rather than overflow the page |
| Ablation | 3.4M | 0.88 | No `aux` loss |

A wide 8-column table:

| Session | Subject | Trials | Hit rate | RT mean (ms) | RT sd (ms) | Gaze error (deg) | Comment |
|---|---|---|---|---|---|---|---|
| 2026-01-01 | monkey_A | 1240 | 0.874 | 412.3 | 88.1 | 1.21 | good session, stable tracking |
| 2026-01-02 | monkey_A | 980 | 0.851 | 430.9 | 92.4 | 1.35 | brief calibration drift |
| 2026-01-03 | monkey_B | 1502 | 0.902 | 398.7 | 75.0 | 0.98 | best session so far |
| 2026-01-04 | monkey_B | 1100 | 0.889 | 405.2 | 80.3 | 1.10 | — |

## Code and Quotes

```python
def f(x: int) -> int:
    """Doc <with> & special chars."""
    return x * 2  # a very long comment that goes on and on and on and on and on past the margin
```

> This is a blockquote with *emphasis* and `code`.
> It spans two lines.
>
> And a second paragraph.

---

## Figures

![Figure 1: Damped sinusoids for $k \in \{0,1,2\}$; colours by *k*.](test_fig.png)

![Missing image test](does_not_exist.png)

### A subsection

Text after the figure. See <https://github.com> or https://jazlab.github.io/tabletop/.

\pagebreak

## Appendix

Final page after an explicit page break.
