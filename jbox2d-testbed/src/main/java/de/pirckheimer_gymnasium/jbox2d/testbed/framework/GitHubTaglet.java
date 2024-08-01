package de.pirckheimer_gymnasium.jbox2d.testbed.framework;

import com.sun.source.doctree.DocTree;
import jdk.javadoc.doclet.Taglet;
import com.sun.source.doctree.UnknownBlockTagTree;
import javax.lang.model.element.Element;
import java.util.List;
import java.util.Set;

public class GitHubTaglet implements Taglet
{
    private static final String NAME = "box2d";

    protected String genLink(String text)
    {
        String relativePath = text;
        String display = text;
        int firstSpace = text.indexOf(' ');
        if (firstSpace != -1)
        {
            relativePath = text.substring(0, firstSpace);
            display = text.substring(firstSpace).trim();
        }
        return String.format("<a href='%s%s'>%s</a>", "", relativePath,
                display);
    }

    @Override
    public String getName()
    {
        return NAME;
    }

    @Override
    public Set<Location> getAllowedLocations()
    {
        return Set.of(Location.METHOD, Location.TYPE);
    }

    @Override
    public boolean isInlineTag()
    {
        return false;
    }

    @Override
    public String toString(List<? extends DocTree> tags, Element element)
    {
        if (tags.isEmpty())
        {
            return null;
        }
        StringBuilder buf = new StringBuilder(String
                .format("<dl><dt><span class=\"strong\">%s</span></dt>", ""));
        for (DocTree tag : tags)
        {
            String text = ((UnknownBlockTagTree) tag).getContent().get(0)
                    .toString();
            buf.append("<dd>").append(genLink(text)).append("</dd>");
        }
        return buf.toString();
    }
}
